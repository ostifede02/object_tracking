# Import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
from script.kalman_filter.kalman_filter import KalmanFilter  # Assuming there's a KalmanFilter class defined in the specified location


# Define time step and standard deviations for acceleration and measurement
dt = 0.1
std_acc = 0.25     # Standard deviation of acceleration (m/s^2)
std_meas = 1.2     # Standard deviation of measurement (m)

# Define state space matrices for the Kalman filter
A = np.array([[1, dt], [0, 1]])
B = np.array([[(dt**2)/2], [dt]])
C = np.array([[1,0]])

Q = np.array([[(dt**4)/4, (dt**3)/2],
               [(dt**3)/2, dt**2]]) * std_acc**2
R = std_meas**2
P = np.eye(A.shape[0])
x = np.array([[0],[0]])
u = 2


# Generate time array
t = np.arange(0, 100, dt)

# Define a model track (in this case, a linear function)
real_track = 0.1 * t**2 + 12

# Create a KalmanFilter object
kf = KalmanFilter(A, B, C, Q, R, x, u, dt)

# Lists to store measurements and predictions
predictions = []
measurements = []

# Iterate over the real track to simulate the Kalman filter process
for x_real in real_track:
    # Measurement
    x_measured = kf.C * x_real + np.random.normal(0, 10)  # Assuming the KalmanFilter class uses @ for matrix multiplication
    
    x_predicted = kf.predict()  # Predict the next state
    kf.update(x_measured.item(0))  # Update the Kalman filter with the measured value

    # Append to lists for plotting
    measurements.append(x_measured.item(0))
    predictions.append(x_predicted.item(0))


# **** PLOT ****
# Plot the results
fig = plt.figure()
fig.suptitle('Kalman filter for tracking a moving object in 1-D', fontsize=16)
plt.plot(t, measurements, label='Measurements', color='b', linewidth=0.5)
plt.plot(t, np.array(real_track), label='Real Track', color='g', linewidth=1.5)
plt.plot(t, np.squeeze(predictions), label='Kalman Filter Prediction', color='r', linewidth=1.5)
plt.xlabel('Time (s)', fontsize=14)
plt.ylabel('Position (m)', fontsize=14)
plt.legend()
plt.show()
