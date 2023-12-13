# Object tracking using Kalman filter and aruco markers
This project aims is to learn and implement a Kalman filter for object tracking. The goal is to develop a model to track an object and predict it's future position and velocity.

This project will finally be implemented in my thesis project: [pick and place delta robot](https://github.com/ostifede02/2dr_ws).


## state space model
The state space model describes the kinematics of a moving object on a conveyor belt.

The state space equations are given by:

$\dot{x} = Ax + Bu$

$y = Cx$

The states for this model are the $x$ and $y$ position of the object and it's velocity. We don't have any control on the system, since we will just observe it.

![x,u](/sources/images/matrices/x_u.png)

The equations of motion along the $x$ and $y$ coordinates are given by:

$x_k = x_{k-1} + \dot{x_{k-1}} Δt + 1/2\ddot{x_{k-1}} Δt^2 $

$y_k = y_{k-1} + \dot{y_{k-1}} Δt + 1/2\ddot{y_{k-1}} Δt^2 $

$\dot{x_k} = \dot{x_{k-1}} + \ddot{x_{k-1}}Δt$

$\dot{y_k} = \dot{y_{k-1}} + \ddot{y_{k-1}}Δt$

The term $Δt$, is the time between each frame and can be defined as the reciprocal of the acquisition rate of the camera ($fps$):

$Δt = 1/fps$

Rearranging these equations in a matrix form, we get the following $A$ and $B$ matrices.


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

# Tracking with aruco markers
In order to show how the Kalman filter works, two possible applications of it are depicted below. For this experiment the camera detects the position of an aruco marker (using openCV) and apply the Kalman filter to the measured position.

| Symbol         | Description                                                    |
| -------------- | -------------------------------------------------------------- |
| Red Dot        | Measured position                                             |
| Green Circle   | Predicted position from measurement                           |
| Blue Circle    | Recursive predicted position from previous predicted position |

## 1. trajectory prediction

Initially the marker is detected. The predicted position corresponds to the measured position.

![prediction1](/sources/images/prediction/prediction1.png)

In the first frames, the prediction shows a transient behavior.

![prediction2](/sources/images/prediction/prediction2.png)

![prediction3](/sources/images/prediction/prediction3.png)

After some measurements, the Kalman filter shows an accurate prediction of future positions.

![prediction4](/sources/images/prediction/prediction4.png)


## 2. tracking without signal
Let's assume that as the car reaches the tunnel, the Kalman filter has already overcome the initial transient phase.

![tunnel1](/sources/images/tunnel/tunnel_1.png)

In case the camera looses the visibility of the marker, the measurement is replaced by the previous prediciton. That is how the algorithm is able to track the marker, even though not detecting it.

![tunnel2](/sources/images/tunnel/tunnel_2.png)

As the car gets out of the tunnel, the marker is detected again. As we can see, during the transition from predicted and measured position, the red dots slightly overlay. This is because, while the marker is not detected, the prediction gains some error and the car might slow down due to friction.

![tunnel3](/sources/images/tunnel/tunnel_3.png)

After few frames, the kalman filter works as well as before.

![tunnel4](/sources/images/tunnel/tunnel_4.png)


## sources
