#include <iostream>
#include "serial_node/serial_node.hpp"

SerialNode::SerialNode()
: Node("serial_node")
{
  // Leggi i parametri
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 9600);
  this->declare_parameter<std::string>("terminator", "");
  this->declare_parameter<int32_t>("inter_byte_timeout", -1);
  this->declare_parameter<int32_t>("read_timeout_constant", 1000);
  this->declare_parameter<int32_t>("read_timeout_multiplier", 0);
  this->declare_parameter<int32_t>("write_timeout_constant", 1000);
  this->declare_parameter<int32_t>("write_timeout_multiplier", 0);
  this->declare_parameter<int32_t>("sampling_thread", 300);

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("terminator", terminator_);
  this->get_parameter("inter_byte_timeout", inter_byte_timeout_);
  this->get_parameter("read_timeout_constant", read_timeout_constant_);
  this->get_parameter("read_timeout_multiplier", read_timeout_multiplier_);
  this->get_parameter("write_timeout_constant", write_timeout_constant_);
  this->get_parameter("write_timeout_multiplier", write_timeout_multiplier_);
  this->get_parameter("sampling_thread", sampling_thread_);

  timeout_ = serial::Timeout(
        (inter_byte_timeout_ == -1) ? serial::Timeout::max() : inter_byte_timeout_,
        read_timeout_constant_,
        read_timeout_multiplier_,
        write_timeout_constant_,
        write_timeout_multiplier_
    );

  enable_service_ =
    this->create_service<std_srvs::srv::SetBool>(
    "/enable",
    std::bind(
      &SerialNode::enableServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));


}

SerialNode::~SerialNode()
{
  if (serial_read_.joinable()) {
    serial_read_.join();
  }
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
}

void SerialNode::enableServiceCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (request->data && !is_enabled_) {
    // Abilita il nodo

    RCLCPP_INFO(this->get_logger(), "Port name: %s", port_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Terminator: %s", terminator_.c_str());

    //serial_port_ = std::make_unique<serial::Serial>(port_name_, baud_rate_, serial::Timeout::simpleTimeout(1000));
    serial_port_ = std::make_unique<serial::Serial>(port_name_, baud_rate_,timeout_);
    if (!serial_port_->isOpen()) {
      throw std::runtime_error("Failed to open the serial port.");
    }
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/serial_output", rclcpp::QoS(
        10));
    subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/serial_input", rclcpp::QoS(
        10), std::bind(&SerialNode::serialInputCallback, this, std::placeholders::_1));

    serial_read_ = std::thread(&SerialNode::serialReadThread, this);

    //pub_timer_ =this->create_wall_timer(std::chrono::milliseconds(300),std::bind(&SerialNode::pub_timer_callback, this));

    //! Logging macro used to deliver a message to the logging subsystem, INFO level
    RCLCPP_INFO(this->get_logger(), "Publisher initialized");


    is_enabled_ = true;
    response->success = true;
    response->message = "Node enabled.";
  } else if (!request->data && is_enabled_) {
    // Disabilita il nodo
    is_enabled_ = false;
    subscriber_.reset();
    if (serial_read_.joinable()) {
      serial_read_.join();
    }
    publisher_.reset();
    serial_port_->close();
    response->success = true;
    response->message = "Node disabled.";
  } else {
    response->success = false;
    response->message = "Node is already in the requested state.";
  }
}

void SerialNode::serialReadThread()
{
  RCLCPP_INFO(this->get_logger(), "Published message %s", "thread ready");
  while (rclcpp::ok() && is_enabled_) {
    if (serial_port_->available()) {
      std::string result;
      if (!terminator_.empty()) {
        result = serial_port_->readline(65536, terminator_);
      } else {
        result = serial_port_->read(65536);
      }

    /*if (result.size() >= 2 && result[result.size() - 2] == '\r' && result.back() == terminator_.at(0)) {
        result.erase(result.size() - 2);  // Rimuove gli ultimi due caratteri
    }
    else if(result.size() >= 1 && result.back() == terminator_.at(0)){
        result.erase(result.size() - 1);
    }*/
    removeTerminator(result, terminator_);

    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data.insert(msg.data.end(), result.begin(), result.end());
    publisher_->publish(msg);

        std::stringstream ss;
        ss << "Dati da pubblicare: [";
        for (size_t i = 0; i < msg.data.size(); ++i)
        {
            ss << static_cast<int>(msg.data[i]);
            if (i < msg.data.size()-1)
            {
                ss << ", ";
            }
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

      /*std::string message = result;
      RCLCPP_INFO(this->get_logger(), "Published message %s", message.c_str());*/
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sampling_thread_));
  }
}

void SerialNode::serialInputCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{

  std::string data(msg->data.begin(), msg->data.end());
  if (!terminator_.empty()) {
    data += terminator_;
  }
  //RCLCPP_INFO(this->get_logger(),data.c_str());
  serial_port_->write(data);
}

void SerialNode::pub_timer_callback(void)
{
  // Build the new message
  std::string new_data = "Hello ";
  new_data.append(port_name_ + "/" + std::to_string(baud_rate_) + "/" + terminator_ + ".");

  //! Create a new message of the specific interface type
  //! It is better to initialize it as empty as below
  std_msgs::msg::String new_msg{};

  //! Populate the data field of the message using its setter method
  new_msg.set__data(new_data);

  //! Publish the new message by calling the publish method of the publisher member
  //publisher_->publish(new_msg);

  // Log something
  std::string message = port_name_ + "/" + std::to_string(baud_rate_) + "/" + terminator_ + ".";
  RCLCPP_INFO(this->get_logger(), "Published message %s", message.c_str());
}

void SerialNode::removeTerminator(std::string& result, const std::string& terminator) {
    size_t terminator_length = terminator.size();

    // Verifica se la lunghezza della stringa è sufficiente per contenere il terminatore
    if (result.size() >= terminator_length) {
        // Controlla se la parte finale della stringa corrisponde al terminatore
        if (result.compare(result.size() - terminator_length, terminator_length, terminator) == 0) {
            result.erase(result.size() - terminator_length);  // Rimuove il terminatore
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
