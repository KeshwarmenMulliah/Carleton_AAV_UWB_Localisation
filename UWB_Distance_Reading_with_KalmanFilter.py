import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from KalmanFilter import *

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False


# ========================
# ========================
# the functions

def setupSerial(baudRate, serialPortName):
    global serialPort

    serialPort = serial.Serial(port=serialPortName, baudrate=baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

    waitForArduino()
# ========================

def sendToArduino(stringToSend):
    # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort

    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8'))  # encode needed for Python3

# ==================
def avg_list(a):
    return sum(a) / len(a)

def recvLikeArduino():
    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8")  # decode needed for Python3

        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True

    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX"

    # ==================


# Function to calculate moving average
def moving_average(data, window_size):
    window = np.ones(int(window_size)) / float(window_size)
    return np.convolve(data, window, 'same')


## KALMAN Filter Implementation


# Perform the filtering
# for i in range(100):
#     # simulate the system
#     x = np.dot(F, x) + np.random.randn(2, 1) * 0.1  # true state
#     z = np.dot(H, x) + np.random.randn() * 0.5  # measured state
#     kf.predict()  # predict the next state
#     kf.update(z)  # update the estimated state with the measurement
#     print(kf.x)  # print the estimated state

def waitForArduino():
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
        msg = recvLikeArduino()
        if not (msg == 'XXX'):
            # print(msg)  # print the distance measurement

            # simulate the system
            x = np.dot(F, x) + np.random.randn(2, 1) * 0.1  # true state
            z = float(msg)  # measured state
            kf.predict()  # predict the next state
            kf.update(z)  # update the estimated state with the measurement
            print(kf.x)  # print the estimated state

            plt.scatter(i, z, c='orange', label='no filter and averaging')  # making a scatter plot
            plt.scatter(i, kf.x[0], c='cyan', label='with Kalman Filter')  # making a scatter plot

            i += 1
            plt.xlabel('Measurement number')
            plt.ylabel('LOS Distance (m)')

            # plt.ylabel('Speed (m/s)')
            if i == 500:
                plt.show(), plt.legend()

            # plt.show()
            # plt.pause(0.00001)


# ====================
# ====================


# the program
setupSerial(115200, "COM6")
