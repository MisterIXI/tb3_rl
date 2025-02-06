import argparse
import socket
import cv2
import numpy as np
from jetcam.csi_camera import CSICamera

# Parse command line argument for color
parser = argparse.ArgumentParser(description='Ball detection script')
parser.add_argument('--color', type=str, required=True, help='Color of the ball to detect (red, blue, green, yellow)')
args = parser.parse_args()
capture_width = 1920

# Initialize camera
cam = CSICamera(
    width=1920,
    height=1080,
    capture_width=capture_width,
    capture_height=1080,
    capture_fps=10
)

# Setup socket connection
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    server_socket.connect(("10.1.1.0", 12345))
except socket.error as e:
    print(f"Socket error: {e}")
    exit(1)

# Define HSV color ranges (adjust these values based on your environment)
color_ranges = {
    'red': [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255]))
    ],
    'blue': (np.array([100, 150, 50]), np.array([140, 255, 255])),
    'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
    'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255]))
}

# Verify valid color
if args.color not in color_ranges:
    print(f"Invalid color. Available options: {', '.join(color_ranges.keys())}")
    exit(1)

# Get the specified color range
selected_color = color_ranges[args.color]

try:
    while True:
        # Get camera frame
        frame = cam.read()
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask based on color
        mask = None
        if isinstance(selected_color, list):
            for lower, upper in selected_color:
                temp_mask = cv2.inRange(hsv, lower, upper)
                mask = temp_mask if mask is None else cv2.bitwise_or(mask, temp_mask)
        else:
            lower, upper = selected_color
            mask = cv2.inRange(hsv, lower, upper)
        
        # Clean up mask with morphological operations
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        center = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Calculate location based on center for Ai Model
                location = (cX - capture_width / 2) / (capture_width / 2)
                    
                # Draw circle and center point
                cv2.circle(frame, (cX, cY), 50, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                                
                # Print center coordinates
                print(f"Ball center: {cX}, {cY}")
                #print(f"Location: {location}")	
                
                # Send coordinates via socket
                try:
                    server_socket.sendall(f"{location}\n".encode())
                except socket.error as e:
                    print(f"Socket send error: {e}")
                    break
        else:
            try:
                server_socket.sendall(b"nan\n")
            except socket.error as e:
                print(f"Socket send error: {e}")
                break
        
        # Display the frame
        #cv2.imshow('Ball Detection', frame)
        cv2.imwrite('detectedBallCenter.jpg', frame)
        # Break loop on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Clean up
    cv2.destroyAllWindows()
    server_socket.close()
    cam.running = False