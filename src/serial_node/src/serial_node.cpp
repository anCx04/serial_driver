/**
 * ROS 2 Serial Comunication Driver Node implementation
 *
 * Andrea Crescenzi <crescenziand@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * September 11, 2024
 */


#include <iostream>
#include "serial_node/serial_node.hpp"

/**
 * @brief Builds a new SerialDriverNode.
 *
 * @param opts ROS 2 node options.
 *
 */
SerialNode::SerialNode()
: Node("serial_node")
{
  // Declare ROS 2 parameters
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 9600);
  this->declare_parameter<std::string>("terminator", "");
  this->declare_parameter<int32_t>("inter_byte_timeout", -1);
  this->declare_parameter<int32_t>("read_timeout_constant", 1000);
  this->declare_parameter<int32_t>("read_timeout_multiplier", 0);
  this->declare_parameter<int32_t>("write_timeout_constant", 1000);
  this->declare_parameter<int32_t>("write_timeout_multiplier", 0);
  this->declare_parameter<bool>("verbose",false);

  // Get parameters and assign them to member variables
  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("terminator", terminator_);
  this->get_parameter("inter_byte_timeout", inter_byte_timeout_);
  this->get_parameter("read_timeout_constant", read_timeout_constant_);
  this->get_parameter("read_timeout_multiplier", read_timeout_multiplier_);
  this->get_parameter("write_timeout_constant", write_timeout_constant_);
  this->get_parameter("write_timeout_multiplier", write_timeout_multiplier_);
  this->get_parameter("verbose",verbose_);

  // Set timeout parameters for serial communication
  timeout_ = serial::Timeout(
    (inter_byte_timeout_ == -1) ? serial::Timeout::max() : inter_byte_timeout_,
    read_timeout_constant_,
    read_timeout_multiplier_,
    write_timeout_constant_,
    write_timeout_multiplier_
  );

  // Create service to enable/disable the serial node
  enable_service_ =
    this->create_service<std_srvs::srv::SetBool>(
    "/enable",
    std::bind(
      &SerialNode::enableServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));
}

/**
 * @brief Cleans up stuff upon node termination.
 */
SerialNode::~SerialNode()
{
  // Clean up the serial read thread if it is still running
  if (serial_read_.joinable()) {
    serial_read_.join();
  }
  // Close the serial port if it is open
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
}

/**
 * @brief enables or disables the serial node
 *
 * @param request Service request to parse.
 * @param response Service response to populate.
 */
void SerialNode::enableServiceCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  // If request is to enable and node is not enabled then I create the pub and sub
  if (request->data && !is_enabled_) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Port name: %s", port_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
      RCLCPP_INFO(this->get_logger(), "Terminator: %s", terminator_.c_str());
    }
    try {
      // Open the serial port with the pre-set param
      serial_port_ = std::make_unique<serial::Serial>(port_name_, baud_rate_, timeout_);
      if (serial_port_->isOpen()) {

        // Create publisher for serial output
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
          "/serial_output", rclcpp::QoS(
            10));

        // Create subscriber for serial input
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
          "/serial_input", rclcpp::QoS(
            10), std::bind(&SerialNode::serialInputCallback, this, std::placeholders::_1));

        // Start the serial read thread to cycling execute the port read
        serial_read_ = std::thread(&SerialNode::serialReadThread, this);

        if (verbose_) {RCLCPP_INFO(this->get_logger(), "Publisher initialized");}

        // Set the response indicating success
        is_enabled_ = true;
        response->success = true;
        response->message = "";
      } else {
        // If failed to open the port
        response->success = false;
        response->message = "Failed to open the serial port";
      }
    } catch (const serial::IOException & e) {
      //std::cerr << "Eccezione catturata: " << e.what() << std::endl;
      response->success = false;
      //response->message = "Failed to open the serial port";
      response->message = e.what();
    }
  } else if (!request->data && is_enabled_) {

    // If request is to disable and node is enabled
    // Disable the node, reset the subscriber and publisher
    is_enabled_ = false;
    subscriber_.reset();

    // Join the serial read thread
    if (serial_read_.joinable()) {
      serial_read_.join();
    }

    // Reset publisher and close the serial port
    publisher_.reset();
    serial_port_->close();
    response->success = true;
    response->message = "";
  } else {

    response->success = false;
    response->message = "Node is already in the requested state.";
  }
}

/**
 * @brief manages reading from the serial port
 */
void SerialNode::serialReadThread()
{
  if (verbose_) {RCLCPP_INFO(this->get_logger(), "thread ready");}

  // Keep reading from the serial port as long as the node is enabled and running
  while (rclcpp::ok() && is_enabled_) {
    std::string result;

    // Read data from the serial port, either using the terminator or a fixed buffer size
    if (!terminator_.empty()) {
      result = serial_port_->readline(65536, terminator_);
    } else {
      result = serial_port_->read(65536);
    }

    if (result.size() > 0) {

      // Remove the terminator if present
      removeTerminator(result, terminator_);

      // Create a UInt8MultiArray message and populate it with the received data
      auto msg = std_msgs::msg::UInt8MultiArray();
      msg.data.insert(msg.data.end(), result.begin(), result.end());

      // Set the layout for the message (1D array)
      msg.layout.dim.resize(1);
      msg.layout.dim[0].label = "byte_array";
      msg.layout.dim[0].size = result.size();
      msg.layout.dim[0].stride = result.size();

      publisher_->publish(msg);

      // Log the published data if verbose mode is enabled
      if (verbose_) {
        std::stringstream ss;
        ss << "Dati da pubblicare: [";
        for (size_t i = 0; i < msg.data.size(); ++i) {
          ss << static_cast<int>(msg.data[i]);
          if (i < msg.data.size() - 1) {
            ss << ", ";
          }
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      }
    }

  }
}

/**
 * @brief Callback for receiving serial input and sending it to the serial port
 *
 * @param msg string to send.
 */
void SerialNode::serialInputCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{

  // Convert the received data into a string and append the terminator if present
  std::string data(msg->data.begin(), msg->data.end());
  if (!terminator_.empty()) {
    data += terminator_;
  }

  serial_port_->write(data);
}

/**
 * @brief Function to remove the terminator from the string
 *
 * @param result string to be reduced
 * @param terminator terminator to remove
 */
void SerialNode::removeTerminator(std::string & result, const std::string & terminator)
{
  size_t terminator_length = terminator.size();

  // If the result has enough characters to include the terminator
  if (result.size() >= terminator_length) {
    // Check if the result ends with the terminator, and remove it
    if (result.compare(result.size() - terminator_length, terminator_length, terminator) == 0) {
      result.erase(result.size() - terminator_length);
    }
  }
}

// Main function to initialize and run the ROS 2 node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                           // Initialize ROS 2
  rclcpp::spin(std::make_shared<SerialNode>());       // Run the SerialNode
  rclcpp::shutdown();                                 // Shutdown ROS 2
  return 0;
}
