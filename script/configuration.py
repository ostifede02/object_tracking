import numpy as np

# Define sampling time
dt = 0.1


# Define the State Transition Matrix A
A = np.matrix([  [1, 0, dt, 0    ],
                [0, 1, 0,  dt   ],
                [0, 0, 1,  0    ],
                [0, 0, 0,  1    ]])

# Define the Control Input Matrix B
B = np.matrix([  [0.5*(dt**2), 0           ],
                [0,         0.5*(dt**2)   ],
                [dt,        0           ],
                [0,         dt          ]])

# Define Measurement Mapping Matrix
C = np.matrix([ [1, 0, 0, 0],
                [0, 1, 0, 0]])

# Initial Process Noise Covariance
std_acc = 1
Q = np.matrix([ [(dt**4)/4,     0,          (dt**3)/2,  0           ],
                [0,             (dt**4)/4,  0,          (dt**3)/2   ],
                [(dt**3)/2,     0,          dt**2,      0           ],
                [0,             (dt**3)/2,  0,          dt**2       ]]) * std_acc**2

#Initial Measurement Noise Covariance
x_std_meas = 0.05
y_std_meas = 0.05
R = np.matrix([ [x_std_meas**2, 0],
                [0,             y_std_meas**2]])


# Define the  control input variables
u = np.zeros((B.shape[1], 1))
# Intial State
x = np.zeros((A.shape[0], 1))