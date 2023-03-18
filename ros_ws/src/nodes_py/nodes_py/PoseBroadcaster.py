import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from uuv_interfaces.msg import Pose



class FramePublisher(Node):
    
    def __init__(self):
        super().__init__('uuv_frame_publisher')
    
        self.tf_broadcaster = TransformBroadcaster(self)

        self.uuv_name = self.declare_parameter('uuv_name', 'uuv').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Pose,
            f'{self.uuv_name}/pose',
            self.handle_uuv_pose,
            10
            )
        
    def handle_uuv_pose(self, msg: Pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.uuv_name
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = msg.z
        
        t.transform.rotation.x = msg.x_quat
        t.transform.rotation.y = msg.y_quat
        t.transform.rotation.z = msg.z_quat
        t.transform.rotation.w = msg.w_quat
        
        self.tf_broadcaster.sendTransform(t)
    
def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()