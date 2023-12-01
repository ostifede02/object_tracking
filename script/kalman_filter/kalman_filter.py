import numpy as np


class KalmanFilter(object):
    def __init__(self, A, B, C, Q, R, x, u, dt):
        self.dt = dt    # time acquisition period
        
        self.A = A      # state matrix
        self.B = B      # control matrix
        self.C = C      # output matrix
        
        self.Q = Q      # process noise matrix
        self.R = R      # measurent noise matrix

        self.P = np.eye(self.A.shape[1])    # error covariance
        self.x = x      # state vector
        self.u = u      # control vector


    def set_x_initial_conditions(self, x):
        self.x = x
        return


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
        return


    def future_predict(self, x_, P_):
        # Update time state
        x = np.dot(self.A, x_) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q
        P = np.dot(np.dot(self.A, P_), self.A.T) + self.Q
        return x, P


    def future_update(self, z, x_, P_):
        # S = H*P*H'+R
        S = np.dot(self.C, np.dot(P_, self.C.T)) + self.R
        
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(P_, self.C.T), np.linalg.inv(S))
        x = np.round(x_ + np.dot(K, (z - np.dot(self.C, x_))))
        I = np.eye(self.C.shape[1])
        P = (I - (K * self.C)) * P_
        return x, P
    
