#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "uuv_interfaces/msg/pose.hpp"

class UUVBroadcaster : public rclcpp::Node
{
    public:
        UUVBroadcaster() : Node("uuv_broadcaster")
        {
            m_uuv_name = this->declare_parameter("uuv_name", "uuv");
            m_sub = this->create_subscription<uuv_interfaces::msg::Pose>(m_uuv_name + "/pose", 10, std::bind(&UUVBroadcaster::pose_callback, this, std::placeholders::_1));
            m_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    
    private:
        void pose_callback(const std::shared_ptr<uuv_interfaces::msg::Pose> msg)
        {
            geometry_msgs::msg::TransformStamped transform;

            transform.header.stamp = this->get_clock()->now();
            transform.header.frame_id = "world";
            transform.child_frame_id = m_uuv_name.c_str();

            transform.transform.translation.x = msg->x;
            transform.transform.translation.y = msg->y;
            transform.transform.translation.z = msg->z;
            transform.transform.rotation.x = msg->x_quat;
            transform.transform.rotation.y = msg->y_quat;
            transform.transform.rotation.z = msg->z_quat;
            transform.transform.rotation.w = msg->w_quat;
            m_broadcaster->sendTransform(transform);
        }

        std::string m_uuv_name;
        rclcpp::Subscription<uuv_interfaces::msg::Pose>::SharedPtr m_sub;
        std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UUVBroadcaster>());
    rclcpp::shutdown();
    return 0;
}