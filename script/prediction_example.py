import cv2
import numpy as np

from script.object_detection.object_detection import ObjectDetection
from script.kalman_filter.kalman_filter import KalmanFilter
import script.configuration as conf

def main():
    # ************  object detection  ************
    video_path = 'sources/videos/test_car.mp4'
    od = ObjectDetection(video_path)
    
    # ************  Kalman filter  ************
    kf = KalmanFilter(conf.A, conf.B, conf.C, conf.Q, conf.R, conf.x, conf.u, conf.dt)

    measured = []
    predicted = []
    future = []
    mp = np.zeros((2, 1))   # measured position
    tp = np.zeros((4, 1))   # predicted position

    while True:
        frame = od.read()      # get frame from imported video
        if frame is None:
            break
        
        frame = cv2.transpose(frame)
        mp = od.pose_estimation(frame)          # measure position of marker
        if mp is None:
            continue                            # if not detected, go to next frame
        
        kf.update(mp)                           # update the Kalman filter with new measurement
        tp = kf.predict()                       # predict next position

        # make a future prediction based on the current prediction
        P_future = np.copy(kf.P)
        x_future = np.copy(tp)
        future.clear()
        
        for i in range(20):
            x_future, P_future = kf.future_update(x_future[0:2], x_future, P_future)
            x_future, P_future = kf.future_predict(x_future, P_future)
            future.append(x_future[0:2])

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