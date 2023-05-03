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
        #self.declare_parameter('P_gain', 150.0)
        #self.declare_parameter('I_gain', 190.0)
        #self.declare_parameter('D_gain', 20.0)
        
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
        self.first_callback = True
        self.last_callback_time = 0
        
        # Data Collection arrays
        self.pitch_array = []
        self.number_of_samples = 0
         
        # Pitch PID control variables
        self.pitch = 0.0
        self.prev_pitch_error = 0.0
        self.pitch_error_int = 0.0
        self.prev_deriv = 0.0
        
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
        
        # Timer for saving data
        save_timer = self.create_timer(4, self.save_data)
    
    def sub_callback(self, msg: Joy):
        # Determine current time
        callback_time = self.get_clock().now().nanoseconds /1e9
        # Retrieve PID gains
        P_gain = self.get_parameter('P_gain').get_parameter_value().double_value
        I_gain = self.get_parameter('I_gain').get_parameter_value().double_value
        D_gain = self.get_parameter('D_gain').get_parameter_value().double_value
        
        # Display Pertinent Data
        #self.get_logger().info(f'Pitch: {self.pitch:.4f}, Prev Error: {self.prev_pitch_error:.4f}, Pitch Error Int: {self.pitch_error_int:.4f}, Prev Deriv: {self.prev_deriv:.4f}, Callback Time: {callback_time - self.last_callback_time:.4f}')
        
        # If first callback, initialize variables
        if self.first_callback:
            self.last_callback_time = callback_time
            self.motorNums = [0,0,0,0]
            self.writeMotor(self.motorNums)
            uuvPose = self.readMsg().split(' ')
            roll, pitch , yaw = self.euler_from_quaternion(float(uuvPose[3]), float(uuvPose[4]), float(uuvPose[5]), float(uuvPose[6]))
            self.pitch = pitch
            self.first_callback = False
        else:    
            # buttons: A, B, X, Y, Left Bumper, Right Bumper
            aButton = msg.buttons[0]
            bumperLeft = msg.buttons[4]
            bumperRight = msg.buttons[5]
        
            if aButton == 1:
                self.turbo = not self.turbo
            scalar = 126 if self.turbo else 64
        
            # axes: Left_x, Left_y, Left_trigger, Right_x, Right_y, Right_trigger
            xLeft = msg.axes[0]
            yLeft = msg.axes[1]
            yRight = msg.axes[4] * np.pi * 3 / 8
        
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

            # Store desired pitch and actual pitch
            self.pitch_array.append([yRight, self.pitch])
            
            # Calculate pitch error
            error = yRight - self.pitch
            # Calculate integral and derivative of pitch error
            self.pitch_error_int += self.trapezoid_integral(error, self.prev_pitch_error, callback_time - self.last_callback_time)
            self.prev_deriv = (error - self.prev_pitch_error) / (callback_time - self.last_callback_time)
            
            # If bumper is pressed, ignore user control
            if bumperLeft == 1 or bumperRight == 1:
                self.motorNums[2] = self.motorNums[3] = -scalar if bumperLeft == 1 else scalar
            # Else, use PID control
            else:
                Prop = P_gain * error
                
                Inte = I_gain * self.pitch_error_int
                
                Deriv = D_gain * self.prev_deriv
                
                frontBackValue = self.saturation(int(Prop + Inte + Deriv), -scalar, scalar)

                # Back motor
                self.motorNums[2] = -frontBackValue
        
                # Front motor
                self.motorNums[3] = frontBackValue

            self.prev_pitch_error = error
        
        # Write motor values to Arduino
        self.writeMotor(self.motorNums)

        # Read UUV pose from Arduino
        uuvPose = self.readMsg().split(' ')
        
        # Publish UUV pose
        pose = Pose()
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.0
        pose.x_quat = float(uuvPose[3])
        pose.y_quat = float(uuvPose[4])
        pose.z_quat = float(uuvPose[5])
        pose.w_quat = float(uuvPose[6])
        self.pose_publisher.publish(pose)
        
        # Calculate roll pitch and yaw
        roll, pitch, yaw = self.euler_from_quaternion(pose.x_quat, pose.y_quat, pose.z_quat, pose.w_quat)
        
        # Store current pitch and time for next callback
        self.pitch = pitch
        self.last_callback_time = callback_time
        
    
    '''Callback function for saving data to csv file'''
    def save_data(self):
        if len(self.pitch_array) >= 1000:
            filename = f'pitch_test_data_{self.number_of_samples}.csv'        
                
            pitch_array = np.array(self.pitch_array.copy())
            
            self.pitch_array = []          

            np.savetxt(filename, pitch_array, delimiter=",", header="Desired, Actual", comments="")
            
            self.number_of_samples += 1
        else:
            return

    '''Method for encoding motor values into bytes and writing to Arduino'''
    def writeMotor(self, nums):
        msg = []
        for num in nums:
            if num >= 0:
                msg.append(32)
            else:
                msg.append(45)
        
            msg.append(abs(num))
        
        self.port.write(bytes(msg))

    '''Method for writing multipurpose messages to Arduino'''
    def writeMsg(self, msg):
        self.port.write(bytes(msg.encode()))

    '''Method for reading multipurpose messages from Arduino'''  
    def readMsg(self):
        msg = self.port.read_until(b';')
        return msg.decode().strip(";")
    
    '''Method for calculating euler angles from quaternions'''
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
    
    '''Method for calculating integral using trapezoid rule'''
    def trapezoid_integral(self, x, x_prev, dt):
        return (x + x_prev) * (dt) / 2
    
    '''Method for saturating values'''
    def saturation(self, value, min, max):
        if value > max:
            return max
        elif value < min:
            return min
        else:
            return value
        

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
    
