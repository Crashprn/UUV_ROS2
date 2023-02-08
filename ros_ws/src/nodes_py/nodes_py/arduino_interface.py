import os
from serial import Serial
import rclpy
import time
from sensor_msgs.msg import Joy

from rclpy.node import Node

class SerialAnalog:
       
    def __init__(self, ser: Serial):
        self.serial = ser
        print(ser)
    
    def writeMsg(self, num):
        msg = []
        if num > 0:
            msg.append(32)
        else:
            msg.append(45)
        
        msg.append(abs(num))
        self.serial.write(bytes(msg))

        
        read = self.serial.read_until()
        return read


class ArduinoInterface(Node):
    
    def __init__(self):
        super().__init__('arduino_interface_node')
        
        portPath = os.path.join("/dev", 'ttyACM0')
        
        ser = Serial(portPath)
        ser.baudrate = 115200
        ser.timeout = 0.1
        self.port = SerialAnalog(ser)
        self.get_logger().info(str(ser.read_until('\n', )))
        

        
        self.subscription = self.create_subscription(Joy, '/joy', self.sub_callback, 10)
    
    
    def sub_callback(self, msg: Joy):
        motorNum = int(msg.axes[1] * 254)
        newMsg = self.port.writeMsg(motorNum)
        
        self.get_logger().info(f'Motor command : {motorNum}, {newMsg.decode()}')
        

def main(args=None):
    rclpy.init()
    
    node = ArduinoInterface()
    time.sleep(2)
    rate = node.create_rate(400)
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    