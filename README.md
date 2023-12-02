# Object tracking using Kalman filter
The goal of this project is to learn and implement a Kalman filter for object tracking. It aims to develop a model to track an object and predict it's future position.

This project will finally be implemented in my thesis project: [pick and place delta robot](https://github.com/ostifede02/2dr).


## state space model
The state space model describes the kinematics of a moving object on a conveyor belt.

The state space equations are given by:

$\dot{x} = Ax + Bu$

$y = Cx$


The states for this model are the $x$ and $y$ position of the object and it's velocity. We don't have any control on the system, since we will just observe it.

![x,u](/sources/images/matrices/x_u.png)

#### State Transition Matrix A
![A](/sources/images/matrices/A.png)

#### Control Input Matrix B
Note that the input matrix is given for a generalization of the problem. In our case, since the input vector $u$ is null, it will not affect the end result.

![B](/sources/images/matrices/B.png)

#### Measurement Mapping Matrix C
We are interested in the position of the object, hence the output of the model are the $x$ and $y$ coordinates.

![C](/sources/images/matrices/C.png)



## Kalman filter
