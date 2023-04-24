import serial
import numpy as np
from KalmanFilter import *


class uwb_localisation():

    def __init__(self):
        self.startMarker = '<'
        self.endMarker = '>'
        self.dataStarted = False
        self.dataBuf = ""
        self.messageComplete = False

        self.serialPort = serial.Serial('COM6', 115200, timeout=1)

    def recvLikeArduino(self):
        global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

        if self.messageComplete == False:

            x = self.serialPort.read().decode("utf-8")  # decode needed for Python3
            if self.dataStarted == True:
                if x != self.endMarker:
                    self.dataBuf = self.dataBuf + x
                else:
                    self.dataStarted = False
                    self.messageComplete = True
            elif x == self.startMarker:
                self.dataBuf = ''
                self.dataStarted = True

        if (self.messageComplete == True):
            self.messageComplete = False
            return self.dataBuf
        else:
            return "XXX"

    def waitForArduino(self):
        # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
        # it also ensures that any bytes left over from a previous message are discarded

        dt = 0.1  # time step (assume constant)
        F = np.array([[1, dt], [0, 1]])  # state transition matrix
        H = np.array([[1, 0]])  # measurement matrix
        Q = np.array([[0.05, 0.05], [0.05, 0.05]])  # process noise covariance
        R = np.array([[0.3]])  # measurement noise covariance
        kf = KalmanFilter(F=F, H=H, Q=Q, R=R)

        # Provide initial state estimate
        x = np.array([[0], [1]])  # initial position and velocity
        kf.x = x

        print("Waiting for Arduino to reset")
        i = 0
        msg = ""
        temp_dist_log = []
        while msg.find("Arduino is ready") == -1:
            msg = self.recvLikeArduino()
            if not (msg == 'XXX'):
                # print(msg)  # print the distance measurement

                # simulate the system
                x = np.dot(F, x) + np.random.randn(2, 1) * 0.1  # true state
                z = float(msg)  # measured state
                kf.predict()  # predict the next state
                kf.update(z)  # update the estimated state with the measurement
                print(kf.x[0][0])  # print the estimated state


# Main Program
uwb = uwb_localisation()
uwb.waitForArduino()
