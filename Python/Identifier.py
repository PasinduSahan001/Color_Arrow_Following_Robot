import time
import cv2
import numpy as np
from collections import Counter
from scipy.spatial import distance
import serial

# Serial setup
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)
time.sleep(2)

# Mapping of directions to serial commands
direction_to_command = {
    "Right": b'L',
    "Up": b'F',
    "Left": b'R',
    "Down": b'B',
    "Stop": b'g',
    "Unknown": b'g',
}


# Function to get the orientation of the contour
def get_orientation(contour, img, midpoint):
    moments = cv2.moments(contour)
    if moments['m00'] != 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
    else:
        cx, cy = 0, 0
    cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
    cv2.line(img, (cx, cy), midpoint, (255, 0, 0), 3)
    angle = np.arctan2(midpoint[1] - cy, midpoint[0] - cx) * 180 / np.pi
    return angle, (cx, cy)


# Function to determine direction from angle
def get_direction(angle):
    if (150 <= angle <= 180) or (-180 <= angle < -150):
        return "Left"
    elif (40 <= angle <= 120) or (0 <= angle < 50):
        return "Up"
    elif (51 <= angle < 90) or (-51 <= angle < -0.01):
        return "Right"
    elif (-120 < angle < -50):
        return "Down"
    else:
        return "Unknown"


# Initialize variables
prev_direction = None
frame_counter = 0
arrow_detected = False
arrow_detected_cont = 0
direction_list = []
waiting_for_response = False  # Flag to indicate waiting for "A"


def detect_arrows(frame):
    global prev_direction, frame_counter, direction_list, arrow_detected, arrow_detected_cont, waiting_for_response

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 120])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask = cv2.bitwise_or(mask_red, mask_black)

    isolated = cv2.bitwise_and(frame, frame, mask=mask)
    gray = cv2.cvtColor(isolated, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)

    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if waiting_for_response:  # Skip arrow detection if waiting for response
        return frame, hsv, mask, gray, blurred, edged

    current_direction = None

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        area = cv2.contourArea(contour)

        if area > 2000:
            contour_points = [tuple(point[0]) for point in approx]
            for idx, point in enumerate(approx):
                cv2.circle(edged, (point[0][0], point[0][1]), 5, (255, 255, 0), -1)
                cv2.putText(edged, str(idx + 1),
                            (point[0][0] + 10, point[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 2)

            if len(contour_points) >= 2:
                dist_matrix = distance.cdist(contour_points, contour_points, 'euclidean')
                np.fill_diagonal(dist_matrix, np.inf)
                min_indices = np.unravel_index(np.argmin(dist_matrix), dist_matrix.shape)

                point1 = contour_points[min_indices[0]]
                point2 = contour_points[min_indices[1]]

                cv2.circle(edged, point1, 5, (255, 0, 255), -1)
                cv2.circle(edged, point2, 5, (255, 0, 255), -1)

                midpoint = ((point1[0] + point2[0]) // 2, (point1[1] + point2[1]) // 2)
                angle, center_mass = get_orientation(contour, edged, midpoint)
                current_direction = get_direction(angle)

                cv2.putText(edged, f"Direction: {current_direction}, Angle: {angle:.2f}",
                            (int(contour[0][0][0]), int(contour[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

            direction_list.append(current_direction)
            frame_counter += 1

            if frame_counter < 10 and prev_direction != current_direction:
                arrow_detected_cont += 1
                if arrow_detected_cont >= 2:
                    print("Stop")

            if prev_direction != current_direction and arrow_detected:
                arrow_detected = False

            if frame_counter >= 2:
                most_common_direction = Counter(direction_list).most_common(1)[0][0]

                if most_common_direction != prev_direction:
                    ser.write(direction_to_command[most_common_direction])
                    print(f"Most common direction: {most_common_direction}")
                    prev_direction = most_common_direction
                    arrow_detected = True
                    waiting_for_response = True  # Start waiting for response

                direction_list = []
                frame_counter = 0

    if not contours:
        if direction_list:
            print("No arrows detected. Stop signal.")
        direction_list = []
        frame_counter = 0

    return frame, hsv, mask, gray, blurred, edged


# Function to read from serial
def read_serial():
    global waiting_for_response
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print(f"Received from serial: {data}")
        if data == "A":
            waiting_for_response = False  # Stop waiting for response
        elif data == "NNN":
            # Reset variables to restart processing
            prev_direction = None
            frame_counter = 0
            arrow_detected = False
            arrow_detected_cont = 0
            direction_list = []
            print("Restarting arrow detection...")

# Initialize the webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
else:
    print("Webcam opened successfully.")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image.")
            break

        frame = cv2.flip(frame, 1)
        frame_with_arrows, hsv, mask, gray, blurred, edged = detect_arrows(frame)
        read_serial()  # Check for incoming serial data

        cv2.imshow('Edged Image', edged)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
