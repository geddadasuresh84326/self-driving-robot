import cv2
import numpy as np

def nothing(x):
    pass

# Initialize camera feed (0 is typically the default webcam/USB camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

cv2.namedWindow("Color Adjustment")

# In OpenCV, Hue range is 0–179, while Saturation and Value are 0–255
cv2.createTrackbar("Lower_H", "Color Adjustment", 0, 179, nothing)
cv2.createTrackbar("Lower_S", "Color Adjustment", 0, 255, nothing)
cv2.createTrackbar("Lower_V", "Color Adjustment", 0, 255, nothing)

cv2.createTrackbar("Higher_H", "Color Adjustment", 179, 179, nothing)
cv2.createTrackbar("Higher_S", "Color Adjustment", 255, 255, nothing)
cv2.createTrackbar("Higher_V", "Color Adjustment", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Optional: Resize frame for lower processing latency
    # frame = cv2.resize(frame, (640, 480))

    # Convert live frame from BGR to HSV
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Read current trackbar positions
    l_h = cv2.getTrackbarPos("Lower_H", "Color Adjustment")
    l_s = cv2.getTrackbarPos("Lower_S", "Color Adjustment")
    l_v = cv2.getTrackbarPos("Lower_V", "Color Adjustment")
    u_h = cv2.getTrackbarPos("Higher_H", "Color Adjustment")
    u_s = cv2.getTrackbarPos("Higher_S", "Color Adjustment")
    u_v = cv2.getTrackbarPos("Higher_V", "Color Adjustment")

    # Define color thresholds
    lower_bound = np.array([l_h, l_s, l_v]) 
    upper_bound = np.array([u_h, u_s, u_v])

    # Generate binary mask and apply to original frame
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
    blue_roi_img = cv2.bitwise_and(frame, frame, mask=mask)

    # Display video windows
    cv2.imshow("MASK", mask)
    cv2.imshow("hsv", hsv_img)
    cv2.imshow("blue_roi_img", blue_roi_img)

    # Press 'ESC' to break
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release camera hardware and close open windows
cap.release()
cv2.destroyAllWindows()