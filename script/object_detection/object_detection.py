import cv2
import numpy as np


class ObjectDetection():
    def __init__(self, video_path):
        self.vid = cv2.VideoCapture(video_path)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)       # create the dictionary for markers type
        return


    def pose_estimation_marker(self, frame):
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(frame, self.dictionary)

        if marker_ids is None:
            return None

        # Calculate the center point of each marker
        marker_corners = corners[0][0]
        cx = int(np.mean(marker_corners[:, 0]))
        cy = int(np.mean(marker_corners[:, 1]))
        mp = np.array([[np.float32(cx)], [np.float32(cy)]])

        return mp


    def draw_canvas(self, frame, measured, predicted, future):
        for measure_point, predict_point in zip(measured, predicted):
            # Draw a circle at the measured point
            cv2.circle(frame, (int(measure_point[0][0]), int(measure_point[1][0])), 4, (0, 0, 255), -1)

            # Draw a circle at the predicted point
            cv2.circle(frame, (int(predict_point[0][0]), int(predict_point[1][0])), 8, (50, 220, 50), 2)

        for future_point in future:
            # Draw a circle at the future points
            cv2.circle(frame, (int(future_point[0][0]), int(future_point[1][0])), 8, (255, 100, 50), 2)

        return frame 
    

    def read(self):
        ret, frame = self.vid.read()
        if not ret:
            return None

        return frame
    

    def show(self, frame):
        cv2.imshow("frame", frame)
        