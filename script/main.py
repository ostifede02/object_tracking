'''
    The aim of this project is to implement a Kalman filter for prediciting the trajectory
    of an object moving on a conveyor belt.
    
    TO DO:
        + make future predictions based on current prediction.   DONE!
        + track the position of the object, even without measurements (object through tunnel)

'''

import cv2
import numpy as np
import time

import os

from object_detection import ObjectDetection
from kalman_filter import KalmanFilter
import configuration as conf


def main():
    # ************  object detection  ************
    video_path = os.path.join(os.path.dirname(__file__), '../sources/videos/test_car.mp4')
    od = ObjectDetection(video_path)
    
    # IP_ADDRESS = "http://10.248.4.234:4747/video"
    # od = ObjectDetection(IP_ADDRESS)

    # ************  Kalman filter  ************
    kf = KalmanFilter(conf.A, conf.B, conf.C, conf.Q, conf.R, conf.x, conf.u, conf.dt)

    measured = []
    predicted = []
    future = []
    mp = np.zeros((2, 1))   # measured position
    tp = np.zeros((4, 1))   # predicted position

    while True:
        # frame = od.read()      # get frame from imported video
        frame = od.read(resize=(0.6, 0.6))      # get frame from imported video
        if frame is None:
            break

        mp = od.pose_estimation(frame)          # measure position of marker
        if mp is None:
            continue                            # if not detected, go to next frame
        
        kf.update(mp)                           # update the Kalman filter with new measurement
        tp = kf.predict()                       # predict next position

        # make a future prediction based on the current prediction
        P_future = np.copy(kf.P)
        x_future = np.copy(tp)
        future.clear()
        
        for i in range(5):
            x_future, P_future = kf.future_update(x_future[0:2], x_future, P_future)
            x_future, P_future = kf.future_predict(x_future, P_future)
            future.append(x_future[0:2])

        # plot states and frame
        measured.append(mp)
        predicted.append(tp)
        frame = od.draw_canvas(frame, measured, predicted, future)
        
        od.show(frame)

        # Check for keypress
        key = cv2.waitKey(0)
        if key == ord('q'): 
            break

    od.vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()