import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
    def __init__(self, A, B, C, D, Q, R, x, u, dt):
        self.dt = dt    # time acquisition period
        
        self.A = A      # state matrix
        self.B = B      # imput matrix
        self.C = C      # output matrix
        self.D = D      # never used before
        
        self.Q = Q      # process noise
        self.R = R      # measurent noise

        self.P = np.eye(self.A.shape[1])    # error covariance
        self.x = x
        self.u = u


    def predict(self):
        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x
    

    def update(self, z):
        # S = H*P*H'+R
        S = np.dot(self.C, np.dot(self.P, self.C.T)) + self.R
        
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.C, self.x))))
        I = np.eye(self.C.shape[1])
        self.P = (I - (K * self.C)) * self.P





dt = 0.1

A = np.matrix([[1, dt], [0, 1]])
B = np.zeros(A.shape[0]) 
C = np.matrix([[1,0]])
D = np.zeros(A.shape[0])
Q = np.zeros(A.shape[0])
R = 1
P = np.eye(A.shape[0])
x = np.matrix([[0],[0]])
u = np.matrix([[0],[0]])






t = np.arange(0, 100, dt)
# Define a model track
real_track = 0.1*((t**2) - t)
u= 2
std_acc = 0.25     # we assume that the standard deviation of the acceleration is 0.25 (m/s^2)
std_meas = 1.2    # and standard deviation of the measurement is 1.2 (m)
# create KalmanFilter object
kf = KalmanFilter(A, B, C, D, Q, R, x, u, dt)
predictions = []
measurements = []
for x in real_track:
    # Mesurement
    z = kf.C * x + np.random.normal(0, 50)
    measurements.append(z.item(0))
    predictions.append(kf.predict()[0])
    kf.update(z.item(0))




# **** PLOT ****
fig = plt.figure()
fig.suptitle('Example of Kalman filter for tracking a moving object in 1-D', fontsize=20)
plt.plot(t, measurements, label='Measurements', color='b',linewidth=0.5)
plt.plot(t, np.array(real_track), label='Real Track', color='y', linewidth=1.5)
plt.plot(t, np.squeeze(predictions), label='Kalman Filter Prediction', color='r', linewidth=1.5)
plt.xlabel('Time (s)', fontsize=20)
plt.ylabel('Position (m)', fontsize=20)
plt.legend()
plt.show()