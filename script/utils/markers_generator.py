import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_size = 400
marker_id = int(input("Enter the marker ID: "))
marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite(f"marker_{marker_id}.png", marker_img)
marker_img = cv2.imread("marker_{}.png".format(marker_id))
cv2.imshow("Marker", marker_img)

cv2.waitKey(0)