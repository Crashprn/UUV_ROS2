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
        
        # Serial Port object 
        self.port= ser
        
        # Right, Left, Back, Front
        self.motorNums = [0,0,0,0]
        
        self.sample_time = 0.015
        
        self.pwm_val = self.declare_parameter('pwm_val', 0).get_parameter_value().integer_value
        self.run_time = self.declare_parameter('run_time', 0).get_parameter_value().integer_value
        self.value_counter = 0
        
        self.pwm_max = 64
        self.value_counter_max = int(self.run_time/self.sample_time)
          
        self.yaw_val = []
        self.prev_yaw = 0
        
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
        
        self.timer = self.create_timer(.015, self.timer_callback)
    
    
    def timer_callback(self):
        
        if self.value_counter >= self.value_counter_max:
            self.timer.destroy()
            self.writeMotor([0,0,0,0])
            self.save_data()
            raise KeyboardInterrupt
        
        # Right Motor
        self.motorNums[0] = self.pwm_val 

        # Left Motor
        self.motorNums[1] = -self.pwm_val
        
        self.value_counter += 1
        
        self.writeMotor(self.motorNums)
        uuvPose = self.readMsg().split(' ')
        
        x_quat = float(uuvPose[3])
        y_quat = float(uuvPose[4])
        z_quat = float(uuvPose[5])
        w_quat = float(uuvPose[6])
        
        roll, pitch, yaw = self.euler_from_quaternion(x_quat, y_quat, z_quat, w_quat)
        if self.yaw_val == []:
            self.prev_yaw = yaw
            self.yaw_val.append(0)
            return
        
        change = self.theta_change(self.prev_yaw, yaw)
        self.yaw_val.append(self.yaw_val[-1] + change)
        
        self.prev_yaw = yaw
        
        self.get_logger().info(f' z: {yaw:.4f}, change: {change:.4f}, angle: {self.yaw_val[-1]:.4f}')
        
    
    def theta_change(self,theta_prev, theta):
        if theta_prev > np.pi/2 and theta < - np.pi/2:
            theta += 2*np.pi
        elif theta_prev < -np.pi/2 and theta > np.pi/2:
            theta -= 2*np.pi
        
        return theta - theta_prev
    
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
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def save_data(self):
        
        filename = f'rotation_test_pwm_{self.pwm_val}.csv'        
                
        save_array = np.zeros(shape=(len(self.yaw_val), 3))
        for row in range(len(self.yaw_val)):
            save_array[row,0] = self.sample_time*row
            save_array[row,1] = self.pwm_val
            save_array[row,2] = self.yaw_val[row]

        np.savetxt(filename, save_array, delimiter=",", header="Time, PWM, Angle", comments="")

        

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