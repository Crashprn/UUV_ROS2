import os
import numpy as np
from serial import Serial
import rclpy
import time
from sensor_msgs.msg import Joy
from rclpy.node import Node
from uuv_interfaces.msg import Pose

class SerialAnalog:
       
    def __init__(self, ser: Serial):
        self.serial = ser
    
    def writeMotor(self, nums):
        msg = []
        for num in nums:
            if num >= 0:
                msg.append(32)
            else:
                msg.append(45)
        
            msg.append(abs(num))
        
        self.serial.write(bytes(msg))
    
    def writeMsg(self, msg):
        self.serial.write(bytes(msg.encode()))
            
    def readMsg(self):
        msg = self.serial.read_until(";")
        return msg.decode().strip(";")


class ArduinoInterface(Node):
    
    def __init__(self, ser):
        # Initializing Node
        super().__init__('arduino_interface_node') 
        self.uuv_name = self.declare_parameter('uuv_name', 'uuv').get_parameter_value().string_value        
        self.port = SerialAnalog(ser)
        
        # Motor related variables
        self.turbo = False
        # Right, Left, Back, Front
        self.motorNums = [0,0,0,0]
        
        # Velocity related variable
        
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        
        self.a_x = 0.0
        self.a_y = 0.0
        self.a_z = 0.0
        
        self.cutoff_freq = 200    
        
        # Sample time related variables
        self.not_first_callback = False
        self.last_callback_time = 0
        
        # Checking if BNO055 is connected
        bno_status = self.port.readMsg()
        if bno_status == '0':
            self.get_logger().info('BNO055 not detected')
            
        # Loop for confirming BNO-055 is calibrated
        while True:
            self.port.writeMsg('1')
            calibration_status = self.port.readMsg().split(' ')
            if calibration_status[0] == 'Calibrated':
                self.get_logger().info('BNO055 calibrated')
                break
            self.get_logger().info(f'Calibration status: {calibration_status}')
            time.sleep(0.5)
        
        # Subscribing to joystick topic
        self.subscription = self.create_subscription(Joy, '/joy', self.sub_callback, 10)
        
        # Publishing pose topic
        self.pose_publisher = self.create_publisher(Pose, f'{self.uuv_name}/pose', 10)
    
    def sub_callback(self, msg: Joy):
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
        
        
        self.port.writeMotor(self.motorNums)
        uuvPose = self.port.readMsg().split(',')
        
        callback_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.not_first_callback:
            time_since_callback = callback_time - self.last_callback_time
            self.a_x = self.digitalFilter(self.a_x, float(uuvPose[0]), time_since_callback, self.cutoff_freq)
            self.a_y = self.digitalFilter(self.a_y, float(uuvPose[1]), time_since_callback, self.cutoff_freq)
            self.a_z = self.digitalFilter(self.a_z, float(uuvPose[2]), time_since_callback, self.cutoff_freq)
            
            self.v_x = self.digitalFilter(self.v_x, self.a_x*time_since_callback, time_since_callback, self.cutoff_freq)
            self.v_y = self.digitalFilter(self.v_y, self.a_y*time_since_callback, time_since_callback, self.cutoff_freq)
            self.v_z = self.digitalFilter(self.v_z, self.a_z*time_since_callback, time_since_callback, self.cutoff_freq)

            self.get_logger().info(f'vx: {self.v_x:.4f}, vy: {self.v_y:.4f}, vz: {self.v_z:.4f}')
            self.get_logger().info(f'ax: {self.a_x:.4f}, ay: {self.a_y:.4f}, az: {self.a_z:.4f}')

        self.not_first_callback = True
        self.last_callback_time = callback_time
        
        pose = Pose()
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.0
        pose.x_quat = float(uuvPose[3])
        pose.y_quat = float(uuvPose[4])
        pose.z_quat = float(uuvPose[5])
        pose.w_quat = float(uuvPose[6])
        self.pose_publisher.publish(pose)
        
    def digitalFilter(self, prevVal: float, newVal: float, sampleTime:float, cutoffFreq: float):
        alpha = np.exp(-1 * cutoffFreq * sampleTime)
        return alpha * prevVal + (1 - alpha) * newVal
        

        

def main(args=None):
    rclpy.init()
    
    portPath = os.path.join("/dev", 'ttyACM0')
        
    ser = Serial(portPath)
    ser.baudrate = 115200
    ser.timeout = 0.2
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
    