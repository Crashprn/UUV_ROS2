#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

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
        m_ser->setDTR(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(22));
        m_ser->setDTR(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::string response = m_ser->read(2);
        response.erase(std::remove(response.begin(), response.end(), ';'), response.end());
        if (response == "0") RCLCPP_ERROR(this->get_logger(), "BNO-055 not detected");

        while (true)
        {
            m_ser->write("1");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::string calibration_status = m_ser->readline(655336, ";");
            calibration_status.erase(std::remove(calibration_status.begin(), calibration_status.end(), ';'), calibration_status.end());
            if (calibration_status == "Calibrated")
            {
                RCLCPP_INFO(this->get_logger(), "Calibration complete");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Calibration status: %s", calibration_status.c_str());
        }

        m_pub = this->create_publisher<uuv_interfaces::msg::Pose>(m_uuv_name + "/pose", 10);
        m_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&ArduinoInterface::joy_callback, this, std::placeholders::_1));
    }

    void joy_callback(const sensor_msgs::msg::Joy& msg)
    {
        // Buttons: A, B, X, Y, LB, RB
        int aButton = msg.buttons[0];
        int bumperLeft = msg.buttons[4];
        int bumperRight = msg.buttons[5];

        // Axes: Left X, Left Y, Left Trigger, Right Trigger, Right X, Right Y
        float xLeft = msg.axes[0];
        float yLeft = msg.axes[1];
        float trigLeft = msg.axes[2];
        float yRight = msg.axes[4];
        float trigRight = msg.axes[5];

        // Checking if turbo is on
        if (aButton) m_turbo = !m_turbo;
        int scalar = m_turbo ? 127 : 64;

        // Normalizing trigger values between 0 and 1
        trigLeft = (1 - trigLeft) / 2;
        trigRight = (1 - trigRight) / 2;

        // Calculating how much to reduce each motor value for turn        
        float leftScalar = xLeft > 0 ? abs(xLeft) : 0;
        float rightScalar = xLeft <= 0 ? abs(xLeft) : 0;
        
        // Finding if left stick is forward or backward
        int direction = yLeft > 0 ? 1 : -1;

        // Right motor
        int rightMotor = static_cast<int>((yLeft * (1-rightScalar) - trigRight * direction) * scalar);
        m_motor_values[0] = rightMotor;

        // Left Motor
        int leftMotor = static_cast<int>((yLeft * (1-leftScalar) - trigLeft * direction) * scalar);
        m_motor_values[1] = leftMotor;

        // Checking if either bumper is pressed
        if (bumperLeft or bumperRight)
        {
            // Straight down if left bumper is pressed or up if right bumper is pressed
            m_motor_values[2] = m_motor_values[3] = (bumperLeft) ? -scalar : scalar;
        }
        else
        {
            // Calculating front and back motor values
            int frontBack = static_cast<int>(yRight * scalar /2);
            // Back motor
            m_motor_values[2] = frontBack;
            // Front motor
            m_motor_values[3] = -frontBack;
        }

        writeMotorVals(m_motor_values);

        std::vector<float> imuVals;

        readIMU(imuVals); 

        uuv_interfaces::msg::Pose pose;
        pose.x = 0.0;
        pose.y = 0.0;
        pose.z = 0.0;
        pose.x_quat = imuVals[3];
        pose.y_quat = imuVals[4];
        pose.z_quat = imuVals[5];
        pose.w_quat = imuVals[6];
        m_pub->publish(pose);
    }

    void writeMotorVals(std::vector<int>& motorValues)
    {
        std::vector<uint8_t> motorVals;
        for (auto& val :motorValues)
        {
            motorVals.push_back((val < 0) ? 45 : 32);
            motorVals.push_back(static_cast<uint8_t>(abs(val)));
        }
        m_ser->write(motorVals);
    }

    void readIMU(std::vector<float>& imuVals)
    {
        std::string response = m_ser->readline(655336, ";");
        response.erase(std::remove(response.begin(), response.end(), ';'), response.end());
        std::stringstream ss(response);
        std::string token;
        while (ss >> token)
        {
            imuVals.push_back(std::stof(token));
        }
    }

    private:
        rclcpp::Publisher<uuv_interfaces::msg::Pose>::SharedPtr m_pub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub;
        std::shared_ptr<serial::Serial> m_ser;
        std::string m_uuv_name;
        bool m_turbo = false;
        std::vector<int> m_motor_values = {0, 0, 0, 0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoInterface>());
    rclcpp::shutdown();
    return 0;
}

