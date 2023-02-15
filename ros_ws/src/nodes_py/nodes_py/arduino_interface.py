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
        self.get_logger().info(str(ser.read_until('\n', )))
        self.turbo = False
        self.motorNums = [0,0,0,0]
        

        
        self.subscription = self.create_subscription(Joy, '/joy', self.sub_callback, 10)
    
    
    def sub_callback(self, msg: Joy):
        # axes: Left_x, Left_y, ....
        xLeft = msg.axes[0]
        yLeft = msg.axes[1]
        
        if msg.buttons[0] == 1:
            self.turbo = not self.turbo
        scalar = 255 if self.turbo else 127
        
        
        leftTurn = abs(xLeft) if xLeft > 0 else 0
        rightTurn = abs(xLeft) if xLeft < 0 else 0
        
        #Right motor
        rightMotor = int((abs(yLeft)* (1-rightTurn) * scalar))
        self.motorNums[0] = -1 * rightMotor if yLeft < 0 else rightMotor
        # Left motor
        leftMotor = int((abs(yLeft) *(1-leftTurn) *scalar))
        self.motorNums[1] = -1 * leftMotor if yLeft < 0 else leftMotor

        self.port.writeMsg(self.motorNums)
        
        self.get_logger().info(f'Left X: {xLeft}, Left Y: {yLeft}, Motor command : {self.motorNums}')
        

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
    