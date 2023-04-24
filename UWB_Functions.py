import math
import numpy as np
import matplotlib.pyplot as plt

# Function that computes the cosine rule
def cosine_rule(a, b, c):
    """
    Calculates the length of the side opposite to the angle C in a triangle given the lengths of the other two sides a and b and the included angle C.

    Parameters:
    a (float): Length of side a
    b (float): Length of side b
    c (float): Included angle C in radians

    Returns:
    tuple: Length of side c and angle opposite to side c in radians
    """
    c = math.sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(c))
    angle_c = math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b))
    return c, angle_c


# Function that calculates the bearing of 3 non collinear points
def get_bearing(point1, point2, point3):
    import math

    # Compute the vectors between the points
    v1 = [point2[0] - point1[0], point2[1] - point1[1]]
    v2 = [point3[0] - point2[0], point3[1] - point2[1]]

    # Compute the dot product and cross product of the vectors
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]

    # Compute the angle between the vectors using the arctangent function
    angle = math.atan2(cross_product, dot_product)

    # Convert the angle to degrees and normalize to the range [0, 360)
    bearing = (math.degrees(angle) + 360) % 360

    # Print the resulting bearing
    print("Bearing between point1 and point3:", bearing)

    return bearing


# Function that computes the bearing given the coordinates of the tags and anchors
def compute_bearing(point1, point2, distance, tag):
    # Compute the midpoint between the two points on the car
    midpoint = ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)

    # Compute the unit vector in the direction of the line connecting the two points
    v = ((point2[0] - point1[0]) / distance, (point2[1] - point1[1]) / distance)

    # Compute the vector between the midpoint and the tag
    w = (tag[0] - midpoint[0], tag[1] - midpoint[1])

    # Compute the angle between the vectors using atan2
    angle = math.atan2(v[1], v[0]) - math.atan2(w[1], w[0])

    # Convert the angle from radians to degrees and normalize to the range [0, 360)
    bearing = math.degrees(angle) % 360

    return bearing


def trilateration(distances):
    # Initialize variables
    n = len(distances)
    A = np.zeros((n - 1, 2))
    b = np.zeros((n - 1, 1))

    # Calculate A and b matrices
    for i in range(1, n):
        A[i - 1, 0] = 2 * (distances[0] - distances[i])
        A[i - 1, 1] = 2 * (distances[1] - distances[i])
        b[i - 1] = distances[0] ** 2 - distances[i] ** 2 - distances[1] ** 2 + distances[i + 1] ** 2

    # Calculate the solution using least squares
    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    return x.flatten()


def check_negative_diff(lst):
    n = len(lst)
    m = 0
    for i in range(1, n):
        if lst[i] - lst[i - 1] < 0:
            m += 1
        else:
            pass
    if m > n / 2:
        return 1
    else:
        return 0


def check_positive_diff(lst):
    n = len(lst)
    m = 0
    for i in range(1, n):
        if lst[i] - lst[i - 1] > 0:
            m += 1
        else:
            pass
    if m > n / 2:
        return 1
    else:
        return 0


def threat_assessement(bearing, distance1_list,ax, STOP= False):
    ax.set_title(
        f"Bearing of Tag rel. to front is {bearing:.2f}")
    if abs(bearing) < 75 and distance1_list[-1] < 4:
        ax.set_title(
            f"Bearing of Tag rel. to front is {bearing:.2f}\n COLLISION THREAT, STOPPING CAR")
        STOP = True
        return STOP
    elif 2 < distance1_list[-1] < 7:
        ax.set_title(
            f"Bearing of Tag rel. to front is {bearing:.2f}\n CAUTION, REDUCING SPEED")
    elif 5 < distance1_list[-1] < 10:
        ax.set_title(
            f"Bearing of Tag rel. to front is {bearing:.2f}\n WARNING, TAG NEARBY")


class KalmanFilter(object):
    def __init__(self, dt, x0, P, A, Q, B, U, H, R):
        self.dt = dt  # time step
        self.x = x0  # initial state vector
        self.P = P  # initial covariance matrix
        self.A = A  # state transition matrix
        self.Q = Q  # process noise covariance
        self.B = B  # input matrix
        self.U = U  # control vector
        self.H = H  # measurement matrix
        self.R = R  # measurement noise covariance

    def predict(self):
        # Predict the state vector and covariance matrix
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.U)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        # Update the state vector and covariance matrix based on the measurement
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(4) - np.dot(K, self.H)), self.P)

    def run(self, measurements):
        # Run the Kalman filter on the given measurements
        predicted_coords = []
        self.predict()
        fig, ax = plt.subplots()
        ax.plot([x[0] for x in measurements], [x[1] for x in measurements], 'rx', label='Measured')
        ax.plot([self.x[0]], [self.x[1]], 'bo', label='Predicted')
        ax.legend(loc='lower right')
        for i, measurement in enumerate(measurements):
            self.update(measurement)
            predicted_coords.append(self.predict()[:2])
            ax.clear()
            ax.plot([x[0] for x in measurements[:i + 1]], [x[1] for x in measurements[:i + 1]], 'rx', label='Measured')
            ax.plot([x[0] for x in predicted_coords], [x[1] for x in predicted_coords], 'b-', label='Predicted')
            ax.plot([self.x[0]], [self.x[1]], 'bo', label='Current Prediction')
            ax.legend(loc='lower right')
            plt.draw()
            plt.pause(0.001)
        return predicted_coords

