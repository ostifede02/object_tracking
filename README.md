# Object tracking using kalman filter
The goal of this project is to learn and implement a Kalman filter for object tracking. It aims to develop a model to track an object and predict it's future position.

This project will finally be implemented in my thesis project: [pick and place delta robot](https://github.com/ostifede02/2dr).


## state space model
The state space model describes the kinematics of a moving object on a conveyor belt.

The state space equations are given by:

$\dot{x} = Ax + Bu$

$y = Cx$

$v = \begin{bmatrix} X \\\ Y \end{bmatrix}$

The states for this model are the $x$ and $y$ position of the object and it's velocity. We don't have any control on the system, since we will just observe it.

$x = \begin{bmatrix}
x \\ 
y \\ 
\dot{x} \\ 
\dot{y} 
\end{bmatrix}$ 

$u = \begin{bmatrix}
0 \\
0 \\
\end{bmatrix}$

#### State Transition Matrix A
$A = \begin{bmatrix}
1 & 0 & \text{dt} & 0 \\
0 & 1 & 0 & \text{dt} \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$

#### Control Input Matrix B
Note that the input matrix is given for a generalization of the problem. In our case, since the input vector $u$ is null, it will not affect the end result.

$B = \begin{bmatrix}
1/2 \times (\text{dt}^2) & 0 \\
0 & 1/2 \times (\text{dt}^2) \\
\text{dt} & 0 \\
0 & \text{dt}
\end{bmatrix}$

#### Measurement Mapping Matrix C
We are interested in the position of the object, hence the output of the model are the $x$ and $y$ coordinates.

$C = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}$



## Kalman filter
