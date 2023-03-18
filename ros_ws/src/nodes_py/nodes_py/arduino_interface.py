import os
from serial import Serial
import rclpy
import time
from sensor_msgs.msg import Joy
from rclpy.node import Node
from uuv_interfaces.msg import Pose

class SerialAnalog:
       
    def __init__(self, ser: Serial):
        self.serial = ser
    
    def writeMsg(self, nums):
        msg = []
        for num in nums:
            if num >= 0:
                msg.append(32)
            else:
                msg.append(45)
        
            msg.append(abs(num))
        
        self.serial.write(bytes(msg))
        
    def readMsg(self):
        msg = self.serial.read_until(";")
        return msg.decode().strip(";")


class ArduinoInterface(Node):
    
    def __init__(self, ser):
        super().__init__('arduino_interface_node')
        
        self.uuv_name = self.declare_parameter('uuv_name', 'uuv').get_parameter_value().string_value        

        self.port = SerialAnalog(ser)
        bno_status = self.port.readMsg()
        if bno_status == '0':
            self.get_logger().info('BNO055 not detected')
        
        
        self.turbo = False
        # Right, Left, Back, Front
        self.motorNums = [0,0,0,0]
        
        self.subscription = self.create_subscription(Joy, '/joy', self.sub_callback, 10)
        self.pose_publisher = self.create_publisher(Pose, f'{self.uuv_name}/pose', 10)
        
        self.not_first_callback = False
        self.last_callback_time = 0
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
    
    
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
        
        
        self.port.writeMsg(self.motorNums)
        uuvPose = self.port.readMsg().split(',')
        
        callback_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.not_first_callback:
            time_since_callback = callback_time - self.last_callback_time
            self.get_logger().info(f'x: {uuvPose[0]}, y: {uuvPose[1]}, z: {uuvPose[2]}')
            self.x += self.v_x* time_since_callback + 0.5 * float(uuvPose[0])*time_since_callback**2
            self.y += self.v_y* time_since_callback + 0.5 * float(uuvPose[1])*time_since_callback**2
            self.z += self.v_z* time_since_callback + 0.5 * float(uuvPose[2])*time_since_callback**2
            
            self.v_x += float(uuvPose[0])*time_since_callback
            self.v_y += float(uuvPose[1])*time_since_callback
            self.v_z += float(uuvPose[2])*time_since_callback

        self.not_first_callback = True
        self.last_callback_time = callback_time
        
        pose = Pose()
        pose.x = self.x
        pose.y = self.y
        pose.z = self.z
        pose.x_quat = float(uuvPose[3])
        pose.y_quat = float(uuvPose[4])
        pose.z_quat = float(uuvPose[5])
        pose.w_quat = float(uuvPose[6])
        self.pose_publisher.publish(pose)
        

        

def main(args=None):
    rclpy.init()
    
    portPath = os.path.join("/dev", 'ttyACM0')
        
    ser = Serial(portPath)
    ser.baudrate = 115200
    ser.timeout = 0.1
    # Reset Arduino
    ser.setDTR(False)
    time.sleep(0.022)
    ser.setDTR(True)
    
    time.sleep(3) 
    node = ArduinoInterface(ser)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    