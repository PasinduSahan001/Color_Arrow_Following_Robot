import os
import time
import cv2
import numpy as np
from collections import Counter
from scipy.spatial import distance
from picamera2 import Picamera2
import threading

# Create a named pipe (FIFO)
fifo_path = 'mypipe'
if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

# Mapping of directions to serial commands
direction_to_command = {
    "Right": b'L',
    "Up": b'F',
    "Left": b'R',
    "Down": b'B',
    "Stop": b'S',
    "Begin": b'A',
    "End": b'O',
    "Unknown": b'U',
}

# Initialize variables
prev_direction = None
frame_counter = 0
direction_list = []
arrow_detected = False
two_second_counter = 0  # Counter for the 2-second wait
stop_after_delay = False  # Flag to indicate whether to send stop after 2 seconds


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
        return "Down"
    elif (51 <= angle < 90) or (-51 < angle < 0):
        return "Right"
    elif (-120 < angle < -50):
        return "Up"
    else:
        return "Unknown"


print_counter = 0
is_running = False


def detect_arrows(frame):
    global prev_direction, frame_counter, direction_list, arrow_detected, two_second_counter, stop_after_delay, print_counter, is_running

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

    print_counter += 1
    if print_counter == 10:
        is_running = True

    current_direction = None
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:

        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        area = cv2.contourArea(contour)

        if area > 2000:
            print_counter = 0
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

            if frame_counter >= 1:
                if is_running == True:
                    send_command("Stop")
                    print_counter = 0
                    is_running = False
                    prev_direction = None
                    time.sleep(1)

                if frame_counter >= 3:
                    most_common_direction = Counter(direction_list).most_common(1)[0][0]
                    if most_common_direction != prev_direction and most_common_direction != 'Unknown':
                        send_command(most_common_direction)
                        print(f"Most common direction: {most_common_direction}")
                        prev_direction = most_common_direction
                        arrow_detected = True
                        print_counter = 0
                        is_running = False

                        two_second_counter = 60
                        stop_after_delay = True

                    direction_list.clear()
                    frame_counter = 0

    if not contours:

        if direction_list:
            print("No arrows detected. Stop signal.")
        direction_list.clear()
        frame_counter = 0

    if two_second_counter > 0:
        two_second_counter -= 1

    return frame, hsv, mask, gray, blurred, edged


def send_command(direction):
    """Send the corresponding command through the pipe."""
    command = direction_to_command.get(direction, b'U')
    with open(fifo_path, 'w') as fifo:
        fifo.write(command.decode() + '\n')
        print(command)

class CameraThread(threading.Thread):
    def __init__(self, picam2):
        super().__init__()
        self.picam2 = picam2
        self.frame = None
        self.running = True

    def run(self):
        while self.running:
            self.frame = self.picam2.capture_array()

    def stop(self):
        self.running = False

picam2 = Picamera2()
picam2.start()

camera_thread = CameraThread(picam2)
camera_thread.start()

try:
    send_command("Begin")
    while True:
        if camera_thread.frame is not None:
            frame = camera_thread.frame
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Convert to BGR for OpenCV

            frame_with_arrows, hsv, mask, gray, blurred, edged = detect_arrows(frame)

            cv2.imshow('Edged Image', edged)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                send_command("End")
                break

except KeyboardInterrupt:
    send_command("End")
    print("Exiting...")

finally:

    camera_thread.stop()
    camera_thread.join()
    cv2.destroyAllWindows()
    picam2.close()