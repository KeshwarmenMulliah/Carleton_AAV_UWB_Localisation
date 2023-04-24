#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Char, Int32, String  # Import any msg types needed here
import serial
import numpy as np
import re
from gpiozero import DistanceSensor
from gpiozero import RGBLED


'''

F = state transition matrix (STM)
H = measurement matrix
Q = process noise covariance matrix
R = measurement noise covariance matrix
x0 = initial state estimate
B = control input that causes the system to change state , it can be acceleration, 
P = initial state error covariance matrix, describes the uncertainty in the initial state

'''

class KalmanFilter(object):
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None):
        if F is None or H is None: # check if the state transition and measurement matrix is provided
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1] # getting the number of rows and storing it in self.n, why?
        self.m = H.shape[1]

        self.F = F
        self.H = H

        # setting B to zero if B is not defined by the user
        self.B = 0 if B is None else B
        # setting Q to a 2D diagonal array of same size as STM with one in the diagonal
        self.Q = np.eye(self.n) if Q is None else Q
        # setting R to a 2D diagonal array of same size as STM with one in the diagonal
        self.R = np.eye(self.n) if R is None else R
        # setting P to a 2D diagonal array of same size as STM with one in the diagonal
        self.P = np.eye(self.n) if P is None else P
        # setting x to a 2D diagonal array of same size as STM with one in the diagonal
        # should self.x be self.x0 instead? KM
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
# If you are not using nanpy, you can remove the lines that have comments that start with #*

# This function is not necessary, but I usually find it useful - it's just the
# 'map' function for arduino, basically linear interpolation
def myMap(inMin, inMax, outMin, outMax, x):
    return int((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin)

class uwb_localisation(Node): # Change this name to the name of your node/program, and MAKE SURE TO ADD A LINE 
                                     #  FOR YOUR PROGRAM TO THE setup.py FILE IN YOUR PACKAGE UNDER CONSOLE SCRIPTS:
                                     #  (make sure to include a comma at end of each node's line only if another node's line comes after it):
                                     #  Examples:
                                     # 'console_scripts': [
                                     #     "nanpy_and_ros2_template = my_py_pkg.nanpy_and_ros2_template:main",
                                     #     "'YOUR NODE/PROGRAM NAME' = 'YOUR PACKAGE NAME'.'YOUR NODE/PROGRAM NAME':main"
                                     #  ],
    def __init__(self): # Put initializations, setup, and most variables here
        super().__init__("uwb_localisation") # Change this name to the name of your node/program
        self.get_logger().info("Initializing UWB modules, please wait...") # Change this name to the name of your node/program
        # This prints an 'info' level message, levels include info, debug, warning, error, fatal, and 'unset'
        self.startMarker = '<'
        self.endMarker = '>'
        self.dataStarted = False
        self.dataBuf = ""
        self.messageComplete = False
        self.msg1 =""
        self.sensor1 = DistanceSensor(echo=16, trigger=12)
        self.led1 = RGBLED(red=5, green=6, blue=26)

        # self.serialPort =  serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        # subscribers
        # self.brake_sub_ = self.create_subscription(Float64, "cmd_brake", self.cmd_brake_callback, 10)

        # publishers
        self.cmd_throttle_pub_ = self.create_publisher(Float64, "cmd_throttle", 10)
        self.cmd_steer_pub_ = self.create_publisher(Float64, "cmd_steer", 10)
        self.heartbeat_pub_ = self.create_publisher(Bool, "controller_heartbeat", 10) # Publisher for a heartbeat
        self.cmd_brake_pub_ = self.create_publisher(Float64, "cmd_brake", 10)

        # This is an example of a timer - basically a way to call a function in the class every x seconds (x=0.1s here)
        self.timer_example_ = self.create_timer(0.005, self.timer_function)
        self.heartbeat_timer_ = self.create_timer(0.0005, self.send_heartbeat) # Timer for sending heartbeats

        self.get_logger().info("UWB localisation script is ready") # Change this name to the name of your node/program

# Add all of your functions below here in the class - if you want it to run 'constantly', set up a timer to call it however often you want:
    
    # Example of a function that is set to run every 0.1s (from above)
    # Includes publishing a message
    def timer_function(self):
        # The following 3 lines should be used to publish data (not strictly in timer function, just here for example):
        msg = Float64() # Change this to your topic's message type

        def recvLikeArduino():
            global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

            startMarker = '<'
            endMarker = '>'
            dataStarted = False
            dataBuf = ""
            messageComplete = False

            serialPort =  serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            serialPort.reset_input_buffer()
            serialPort.reset_output_buffer()

            if  messageComplete == False:
            
                x = serialPort.read(6).decode("utf-8")  # decode needed for Python3
                # print(f'distance is {x}')
                m = re.search('[0-9].[0-9]*',x) 
                # print(m.group(0))
                return m.group(0)

        
        # waitForArduino()
        distance = self.sensor1.distance * 100
        print(f'Distance from ultrasonic sensor: {distance}')
        y = recvLikeArduino()
        dt = 0.05  # time step (assume constant)
        F = np.array([[1, dt], [0, 1]])  # state transition matrix
        H = np.array([[1, 0]])  # measurement matrix
        Q = np.array([[0.05, 0.05], [0.05, 0.05]])  # process noise covariance
        R = np.array([[0.3]])  # measurement noise covariance
        kf = KalmanFilter(F=F, H=H, Q=Q, R=R)

        # Provide initial state estimate
        x = np.array([[0], [1]])  # initial position and velocity
        kf.x = x

        # simulate the system
        x = np.dot(F, x) + np.random.randn(2, 1) * 0.1  # true state
        z = float(y)  # measured state
        kf.predict()  # predict the next state
        kf.update(z)  # update the estimated state with the measurement
        print(kf.x[0][0])  # print the estimated state

        uwb_distance = kf.x[0][0]
        if uwb_distance < 2:
            print("Less than fucking 2")

        print(f'distance is {uwb_distance}')

        if uwb_distance <= 1 and distance < 100: 
            msg.data = 100.0# throttle for test
            print("car is at the back ")
            self.led1.color = (1, 0, 0)  # red
            self.cmd_throttle_pub_.publish(msg) # Publish the message to the motors

        elif uwb_distance <=1 and distance ==100 :# if tag is too close, apply brakes
            msg.data = 0.0
            self.led1.red = 1
            self.led1.color = (1, 0, 0)  # red
            self.cmd_throttle_pub_.publish(msg)
        elif 1 < uwb_distance < 2:
            self.led1.color = (1, 1, 0)  # yellow
            msg.data = 50.0
            self.cmd_throttle_pub_.publish(msg) # Publish the message to the motors
        else:
            self.led1.color = (0, 1, 0)  # green
            msg.data = 20.0 # stop turning
            self.cmd_steer_pub_.publish(msg) # Publish the message to the servos
            msg.data = 100.0 # go foward
            self.cmd_throttle_pub_.publish(msg) # Publish the message to the motors

    def send_heartbeat(self): # Function that will send heartbeats, don't change this if your program requires a heartbeat
                              #  but you can remove it if your program stopping is not a safety concern to the vehicle
        msg = Bool()
        msg.data = True
        self.heartbeat_pub_.publish(msg)
        

def main(args=None): # Your main function
    rclpy.init(args=args) # Leave this as is

    node = uwb_localisation() # Change this name to the name of your node/program
    try:
        rclpy.spin(node) # Leave as is, this keeps you node alive even if nothing is happening
    except KeyboardInterrupt:
        pass # Leave as is, program will leave node spinning 'loop' and run what's next
    finally: # Put any shutdown code here
        msg = Float64()
        msg.data = 0.0 # stop turning
        node.cmd_steer_pub_.publish(msg) # Publish the message to the servos
        