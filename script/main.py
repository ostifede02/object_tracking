import cv2
import numpy as np

# DROID_CAM_IP_ADDRESS = "http://10.248.24.172:4747/video/"
vid = cv2.VideoCapture("test_video/test_1.mp4")
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)       # create the dictionary for markers type


# ************  Kalman filter  ************
measured = []
predicted = []
mp = np.array((2, 1), np.float32)   # measured position
tp = np.zeros((4, 1), np.float32)   # predicted position

kalman_fil = cv2.KalmanFilter(4, 2)
kalman_fil.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman_fil.transitionMatrix = np.array(
    [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32
)
kalman_fil.processNoiseCov = (
    np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    * 0.03
)


# ************  utility functions  ************
def pose_estimation(frame, dictionary):
    corners, marker_ids, rejected = cv2.aruco.detectMarkers(frame, dictionary)

    # Check if markers are detected
    if marker_ids is not None:
        # Calculate the center point of each marker
        cx = int((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4)
        cy = int((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4)
        mp = np.array([[np.float32(cx)], [np.float32(cy)]])
        return mp        
    else:
        return None
    

def pose_prediction(kf, mp):
    kf.correct(mp)
    tp = kf.predict()
    return tp


def draw_canvas(frame, measured, predicted):

    for i in range(len(measured)):
        # Draw a circle at the center point
        cv2.circle(frame, (int(measured[i][0][0]), int(measured[i][1][0])), 6, (0, 0, 255), -1)
        cv2.circle(frame, (int(predicted[i][0][0]), int(predicted[i][1][0])), 10, (255, 0, 0), 2)
            
    return frame


is_first = True

while True:
    ret, frame = vid.read()
    
    frame = cv2.resize(frame, (int(frame.shape[1]*0.6), int(frame.shape[0]*0.6)))
    frame = frame[0:760, 130:280]
    
    mp = pose_estimation(frame, dictionary)
    if mp is None:
        print("Object identified = None")
        continue

    tp = pose_prediction(kalman_fil, mp)
    

    # plot states and frame
    measured.append(mp)
    predicted.append(tp)
    frame = draw_canvas(frame, measured, predicted)
    
    frame = cv2.transpose(frame)
    cv2.imshow("frame", frame)

    # Check for keypress
    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'): 
        break
    elif key == ord(' '):
        continue


vid.release()
cv2.destroyAllWindows()