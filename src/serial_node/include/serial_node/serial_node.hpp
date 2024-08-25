#ifndef SERIAL_NODE_HPP_
#define SERIAL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <serial/serial.h>
#include <string>
#include <memory>
#include <thread>


class SerialNode : public rclcpp::Node
{
public:
   SerialNode();
   ~SerialNode();

private:
    std::string port_name_;
    int baud_rate_;
    std::string terminator_;

    int32_t inter_byte_timeout_ ;
    int32_t read_timeout_constant_ ;
    int32_t read_timeout_multiplier_ ;
    int32_t write_timeout_constant_ ;
    int32_t write_timeout_multiplier_ ;
    serial::Timeout timeout_ ;

    int32_t sampling_thread_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    void enableServiceCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response);
    bool is_enabled_=false;

    std::unique_ptr<serial::Serial> serial_port_;
    std::thread serial_read_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    void serialReadThread();

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscriber_;
    void serialInputCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr pub_timer_;
    void pub_timer_callback(void);

    void removeTerminator(std::string& result, const std::string& terminator);

};

#endif  // SERIAL_NODE_HPP_