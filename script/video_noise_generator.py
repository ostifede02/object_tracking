import cv2
import numpy as np

import os



def main():
    # Open the video file
    input_video_path = os.path.join(os.path.dirname(__file__), '../sources/videos/test_car.mp4')
    cap = cv2.VideoCapture(input_video_path)

    # Get video properties
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    ret, frame = cap.read()      # get frame from imported video
    width = frame.shape[1]
    height = frame.shape[0]
    
    # Create VideoWriter object to save the output video
    output_video_path = os.path.join(os.path.dirname(__file__), '../sources/videos/test_car_noise.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))


    while cap.isOpened():
        ret, frame = cap.read()      # get frame from imported video
        if not ret:
            break
        
        # add noise to video
        out_frame = np.copy(frame)
        col_start, col_end = 300, 500
        out_frame[0:height, col_start:col_end] = [255, 75, 50]

        # Add text to the frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "It's a tunnel."
        textsize = 0.9

        # Calculate the position to center the text
        textX = col_start + 5
        textY = 25

        # Put the text on the frame
        cv2.putText(out_frame, text, (textX, textY), font, textsize, (255, 255, 255), 1, cv2.LINE_AA)

        out.write(out_frame)

        cv2.imshow("frame", frame)
        cv2.imshow("out frame", out_frame)

        # Check for keypress
        key = cv2.waitKey(1)
        if key == ord('q'): 
            break


    out.release()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

