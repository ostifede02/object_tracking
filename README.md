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
The Kalman filter is implemented with a KalmanFilter class. It has been written from skratch in order to have more control on the states variables, which resulted to be an issue using other libraries. 

In order to show how the Kalman filter works, two examples are provided below.


## tracking with aruco markers
#### legend
| Symbol         | Description                                                    |
| -------------- | -------------------------------------------------------------- |
| Red Dot        | Measured position                                             |
| Green Circle   | Predicted position from measurement                           |
| Blue Circle    | Recursive predicted position from previous predicted position |

### trajectory prediction

In the first frame the marker is detected, the predicted position corresponds to it.
![prediction1](/sources/images/prediction/prediction1.png)

In the first frames, the prediction shows a transient behavior.
![prediction2](/sources/images/prediction/prediction2.png)

![prediction3](/sources/images/prediction/prediction3.png)

After some measurements, the Kalman filter shows an accurate prediction of future position.
![prediction4](/sources/images/prediction/prediction4.png)


### tracking without signal
Let's assume that as the car reaches the tunnel, the Kalman filter has already overcome the initial transient phase.
![tunnel1](/sources/images/tunnel/tunnel_1.png)

If the camera loose the the position of the marker, the measurement is replaced by the previous prediciton. That is why the algorithm is able to track the marker, even without detecting it.
![tunnel2](/sources/images/tunnel/tunnel_2.png)

As the car get's out of the tunnel.
![tunnel3](/sources/images/tunnel/tunnel_3.png)
![tunnel4](/sources/images/tunnel/tunnel_4.png)