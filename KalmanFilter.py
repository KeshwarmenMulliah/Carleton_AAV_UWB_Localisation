import numpy as np
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