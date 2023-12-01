import cv2
import numpy as np

import os

from object_detection import ObjectDetection
from kalman_filter import KalmanFilter
import configuration as conf


# ************  object detection  ************
video_path = os.path.join(os.path.dirname(__file__), '../sources/videos/test_car.mp4')
od = ObjectDetection(video_path)

# ************  Kalman filter  ************
kf = KalmanFilter(conf.A, conf.B, conf.C, conf.Q, conf.R, conf.x, conf.u, conf.dt)

measured = []
predicted = []
mp = np.zeros((2, 1), np.float32)   # measured position
tp = np.zeros((4, 1), np.float32)   # predicted position


def main():

    while True:
        frame = od.read((0.6, 0.6))   
        mp = od.pose_estimation(frame)
        if mp is None:
            # print("Object identified = None")
            continue

        kf.update(mp)
        tp = kf.predict()

        # plot states and frame
        measured.append(mp)
        predicted.append(tp)
        frame = od.draw_canvas(frame, measured, predicted)
        
        od.show(frame)

        # Check for keypress
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'): 
            break
        elif key == ord(' '):
            continue


    od.vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()