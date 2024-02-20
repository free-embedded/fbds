# Import the necessary modules
import cv2
import numpy as np
import cv2.aruco as aruco
import serial

# Setup serial connection
ser = serial.Serial('COM6', 9600)

# Define the Aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# Create the detector parameters
parameters =  aruco.DetectorParameters()

# Initialize the video capture
cap = cv2.VideoCapture(0)

# Define the width and height of the camera frame
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Define the center of the frame
center_x = width / 2
center_y = height / 2

# Define the tolerance for the centering
tolerance = 50

def read_from_esp32():
    if ser.inWaiting() > 0:  # Check if data is available
        incoming_message = ser.readline().decode('utf-8').rstrip()  # Read the incoming message and decode it
        print(f"Message from ESP32: {incoming_message}")

# Define a function to print the movement instructions
def print_movement(x, y):
    # Check if the marker is close enough to the center
    if abs(x - center_x) < tolerance and abs(y - center_y) < tolerance:
        print("The marker is centered.")
        ser.write("centered\n".encode())
    else:
        # Print the horizontal movement
        if x < center_x - tolerance:
            print("Go left.")
            ser.write("go left\n".encode())
        elif x > center_x + tolerance:
            print("Go right.")
            ser.write("go right\n".encode())
        # Print the vertical movement
        if y < center_y - tolerance:
            print("Go up.")
            ser.write("go up\n".encode())
        elif y > center_y + tolerance:
            print("Go down.")
            ser.write("go down\n".encode())

# Loop until the user presses 'q'
while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    # Now, let's also check for messages from the ESP32
    read_from_esp32()
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect the Aruco markers in the frame
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # If at least one marker is detected
    if len(corners) > 0:
        # Draw the detected markers on the frame
        aruco.drawDetectedMarkers(frame, corners, ids)
        # Get the coordinates of the first marker's center
        x = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
        y = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
        # Print the movement instructions every 100ms
        print_movement(x, y)
        cv2.waitKey(100)
    else:
        print("No marker detected.")
        ser.write("no target\n".encode())
        cv2.waitKey(100)
    # Show the frame
    cv2.imshow("Frame", frame)
    # Check if the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the windows
cap.release()
cv2.destroyAllWindows()
