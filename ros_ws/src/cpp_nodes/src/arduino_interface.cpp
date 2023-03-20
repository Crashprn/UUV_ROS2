#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "uuv_interfaces/msg/pose.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"


class ArduinoInterface : public rclcpp::Node
{
    public:
    ArduinoInterface():
    Node("arduino_interface")
    {
        m_uuv_name = this->declare_parameter("uuv_name", "uuv");

        m_ser = std::make_shared<serial::Serial>("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
        if (!m_ser->isOpen()) RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::string response = m_ser->read(2);
        response.erase(std::remove(response.begin(), response.end(), ';'), response.end());

        while (true)
        {
            m_ser->write("1");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::string calibration_status = m_ser->readline(655336, ";");
            calibration_status.erase(std::remove(calibration_status.begin(), calibration_status.end(), ';'), calibration_status.end());
            RCLCPP_INFO(this->get_logger(), "Calibration status: %s", calibration_status.c_str());
        }

        m_pub = this->create_publisher<uuv_interfaces::msg::Pose>(m_uuv_name + "/pose", 10);
        m_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&ArduinoInterface::joy_callback, this, std::placeholders::_1));
    }

    void joy_callback(const sensor_msgs::msg::Joy& msg)
    {
        int aButton = msg.buttons[0];
        int bumperLeft = msg.buttons[4];
        int bumperRight = msg.buttons[5];

        if (aButton){m_turbo = !m_turbo;}
        int scalar = m_turbo ? 127 : 63;


    }

    private:
        rclcpp::Publisher<uuv_interfaces::msg::Pose>::SharedPtr m_pub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub;
        std::shared_ptr<serial::Serial> m_ser;
        std::string m_uuv_name;
        bool m_turbo = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoInterface>());
    rclcpp::shutdown();
    return 0;
}

