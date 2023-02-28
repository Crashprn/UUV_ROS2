import os
from serial import Serial
import rclpy
import time
from sensor_msgs.msg import Joy
import threading
from rclpy.node import Node

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
        #returnMsg = self.serial.read_until(';')
        #return msg, returnMsg


class ArduinoInterface(Node):
    
    def __init__(self, ser):
        super().__init__('arduino_interface_node')
        

        self.port = SerialAnalog(ser)
        self.turbo = False
        # Right, Left, Back, Front
        self.motorNums = [0,0,0,0]
        

        
        self.subscription = self.create_subscription(Joy, '/joy', self.sub_callback, 10)
    
    
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
        
        
        #sent, returned = self.port.writeMsg(self.motorNums)
        self.port.writeMsg(self.motorNums)
        
        #self.get_logger().info(returned.decode())
        

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
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    