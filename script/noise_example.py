import cv2
import numpy as np

from script.object_detection.object_detection import ObjectDetection
from script.kalman_filter.kalman_filter import KalmanFilter
import script.configuration as conf
 
def main():
    # ************  object detection  ************
    video_path = 'sources/videos/test_car_noise.mp4'
    od = ObjectDetection(video_path)
    
    # ************  Kalman filter  ************
    kf = KalmanFilter(conf.A, conf.B, conf.C, conf.Q, conf.R, conf.x, conf.u, conf.dt)

    measured = []
    predicted = []
    future = []
    mp = np.zeros((2, 1))   # measured position
    tp = np.zeros((4, 1))   # predicted position

    is_first_detected = False
    while True:
        frame = od.read()      # get frame from imported video
        if frame is None:
            break
        
        frame = cv2.transpose(frame)
        mp = od.pose_estimation(frame)          # measure position of marker
        
        if mp is None:              # if any marker is detected
            if is_first_detected:   # if one marker has been detected, but lost of view
                mp = tp[0:2]
            else:
                continue            # if any marker have been ever detected

        is_first_detected = True
        
        kf.update(mp)                           # update the Kalman filter with new measurement
        tp = kf.predict()                       # predict next position


        # plot states and frame
        measured.append(mp)
        predicted.append(tp)
        frame = od.draw_canvas(frame, measured, predicted, future)
        
        frame = cv2.transpose(frame)
        od.show(frame)

        # Check for keypress
        key = cv2.waitKey(0)
        if key == ord('q'): 
            break

    od.vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()