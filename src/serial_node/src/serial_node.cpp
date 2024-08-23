#include <iostream>
#include "serial_node/serial_node.hpp"

SerialNode::SerialNode()
: Node("serial_node"), is_enabled_(false)
{
  // Leggi i parametri
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 9600);
  this->declare_parameter<std::string>("terminator", "");

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("terminator", terminator_);

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

    serial_port_ = std::make_unique<serial::Serial>(
      port_name_, baud_rate_, serial::Timeout::simpleTimeout(
        1000));
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

       if (result.size() >= 2 && result[result.size() - 2] == '\r' && result.back() == '\n') {
        result.erase(result.size() - 2);  // Rimuove gli ultimi due caratteri
    }

      auto msg = std_msgs::msg::UInt8MultiArray();
      msg.data.insert(msg.data.end(), result.begin(), result.end());
      publisher_->publish(msg);

      /*  std::stringstream ss;
        ss << "Dati da pubblicare: [";
        for (size_t i = 0; i < msg.data.size(); ++i)
        {
            ss << static_cast<int>(msg.data[i]);
            if (i < msg.data.size() - 1)
            {
                ss << ", ";
            }
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str()); */

      std::string message = result;
      RCLCPP_INFO(this->get_logger(), "Published message %s", message.c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));      // Evita di occupare inutilmente la CPU
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
