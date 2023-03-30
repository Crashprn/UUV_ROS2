import os
import numpy as np
from serial import Serial
import rclpy
import time
from sensor_msgs.msg import Joy
from rclpy.node import Node
from uuv_interfaces.msg import Pose


class ArduinoInterface(Node):
    
    def __init__(self, ser : Serial):
        # Initializing Node
        super().__init__('arduino_interface_node') 
        self.uuv_name = self.declare_parameter('uuv_name', 'uuv').get_parameter_value().string_value
        
        # Declaring proportional, integral, and derivative gains
        self.declare_parameter('P_gain', 0.0)
        self.declare_parameter('I_gain', 0.0)
        self.declare_parameter('D_gain', 0.0)
        
        # Serial Port object 
        self.port= ser
        
        # Motor related variables
        self.turbo = False
        # Right, Left, Back, Front
        self.motorNums = [0,0,0,0]
         
        
        # Sample time related variables
        self.not_first_callback = False
        self.last_callback_time = 0
        
        # Checking if BNO055 is connected
        bno_status = self.readMsg()
        if bno_status == '0':
            self.get_logger().info('BNO055 not detected')
            
        # Loop for confirming BNO-055 is calibrated
        while True:
            self.writeMsg('1')
            calibration_status = self.readMsg().split(' ')
            if calibration_status[0] == 'Calibrated':
                self.get_logger().info('BNO055 calibrated')
                break
            self.get_logger().info(f'Calibration status: {calibration_status}')
            time.sleep(0.1)
        
        # Subscribing to joystick topic
        self.subscription = self.create_subscription(Joy, '/joy', self.sub_callback, 10)
        
        # Publishing pose topic
        self.pose_publisher = self.create_publisher(Pose, f'{self.uuv_name}/pose', 10)
    
    def sub_callback(self, msg: Joy):
        P_gain = self.get_parameter('P_gain').get_parameter_value().double_value
        I_gain = self.get_parameter('I_gain').get_parameter_value().double_value
        D_gain = self.get_parameter('D_gain').get_parameter_value().double_value
        
        self.get_logger().info(f'P: {P_gain}, I: {I_gain}, D: {D_gain}')
        
        # buttons: A, B, X, Y, Left Bumper, Right Bumper
        aButton = msg.buttons[0]
        bumperLeft = msg.buttons[4]
        bumperRight = msg.buttons[5]
        
        if aButton == 1:
            self.turbo = not self.turbo
        scalar = 255 if self.turbo else 127
        
        # axes: Left_x, Left_y, Left_trigger, Right_x, Right_y, Right_trigger
        xLeft = msg.axes[0]
        yLeft = msg.axes[1]
        yRight = msg.axes[4]
        
        # Normalizing trigger values between 0 and 1
        trigLeft = (1 - msg.axes[2]) / 2
        trigRight = (1 - msg.axes[5]) / 2
        
        # Calculating how much to reduce left or right motor output
        leftScaler = abs(xLeft) if xLeft > 0 else 0
        rightScaler = abs(xLeft) if xLeft < 0 else 0
        
        direction = 1 if yLeft > 0 else -1
        
        #Right motor
        rightMotor = int((yLeft * (1-rightScaler) - trigRight * direction) * scalar)
        self.motorNums[0] = rightMotor

        # Left motor
        leftMotor = int((yLeft *(1-leftScaler) - trigLeft* direction) *scalar)
        self.motorNums[1] = leftMotor

        if bumperLeft == 1 or bumperRight == 1:
            self.motorNums[2] = self.motorNums[3] = -scalar if bumperLeft == 1 else scalar
        else:
            # Calculating front and back motor values
            frontBackValue = int(yRight * scalar/2)
            # Back motor
            self.motorNums[2] = frontBackValue
        
            # Front motor
            self.motorNums[3] = -frontBackValue
        
        self.writeMotor(self.motorNums)
        uuvPose = self.readMsg().split(' ')
        
        pose = Pose()
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.0
        pose.x_quat = float(uuvPose[3])
        pose.y_quat = float(uuvPose[4])
        pose.z_quat = float(uuvPose[5])
        pose.w_quat = float(uuvPose[6])
        self.pose_publisher.publish(pose)
        
        callback_time = self.get_clock().now().nanoseconds / 1e9
        
        self.last_callback_time = callback_time
    
    def writeMotor(self, nums):
        msg = []
        for num in nums:
            if num >= 0:
                msg.append(32)
            else:
                msg.append(45)
        
            msg.append(abs(num))
        
        self.port.write(bytes(msg))

    def writeMsg(self, msg):
        self.port.write(bytes(msg.encode()))
            
    def readMsg(self):
        msg = self.port.read_until(b';')
        return msg.decode().strip(";")
        

        

        

def main(args=None):
    rclpy.init()
    
    portPath = os.path.join("/dev", 'ttyACM0')
        
    ser = Serial(portPath)
    ser.baudrate = 115200
    ser.timeout = 1
    # Reset Arduino
    ser.setDTR(False)
    time.sleep(0.022)
    ser.setDTR(True)
    
    time.sleep(2) 
    node = ArduinoInterface(ser)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    