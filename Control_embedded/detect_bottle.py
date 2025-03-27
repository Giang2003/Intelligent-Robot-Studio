import cv2
import numpy as np
import serial
import time

# 1. Kết nối với Arduino qua Serial
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)
time.sleep(2)  # Chờ Arduino khởi động
camera = cv2.VideoCapture(0)
# PID controller variables
kp = 0.5  # Proportional gain
ki = 0.01  # Integral gain
kd = 0.1  # Derivative gain
previous_error = 0
integral = 0

# Distance thresholds
GRIP_DISTANCE = 7.0  # Distance in cm to trigger gripping
SLOW_DISTANCE = 10.0  # Distance in cm to reduce speed
# Verify if the camera opened successfully
if not camera.isOpened():
    print("Error: Could not access the camera.")
    exit()
else:
    print("Camera successfully opened.")



def send_command(command):
    """Send a command to the Arduino."""
    arduino.write(f"{command}\n".encode())
    print(f"Command Sent: {command}")
    # response = arduino.readline().decode().strip()
    # print(f"Response from Arduino: {response}")




# 6. Vòng lặp chính
try:
    while True:
        # Bước 1: Lấy khung hình từ camera
        ret, frame = camera.read()
        if not ret:
            print("Error: Failed to capture frame.")
            continue  # Skip this iteration if frame capture failed
        else:
            print("Frame captured successfully.")
        

        # Get the frame dimensions
        height, width, _ = frame.shape
        # Bước 2: Phát hiện màu xanh
                # Convert the frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        print("Converted frame to HSV.")

        # Define range for green color in HSV
        lower_green = np.array([35, 100, 100])  # Adjust as needed
        upper_green = np.array([85, 255, 255])  # Adjust as needed

        # Create a mask to extract green color
        mask = cv2.inRange(hsv, lower_green, upper_green)
        green_detection = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Lấy contour lớn nhất
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Calculate the center of the green object
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Draw a rectangle and circle at the center of the green object
            cv2.rectangle(green_detection, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(green_detection, (center_x, center_y), 5, (0, 0, 255), -1)
            # Calculate the error between the object center and the frame center
            error = center_x - width // 2
            print(f"Object center: ({center_x}, {center_y}), Error: {error}")

            # PID calculation
            integral += error
            derivative = error - previous_error
            correction = kp * error + ki * integral + kd * derivative
            previous_error = error
            print(f"PID correction: {correction}")
            


            # Bước 3: Đo khoảng cách liên tục
            send_command("DISTANCE")  # Ask Arduino for distance
            distance = arduino.readline().decode().strip()  # Read distance from Arduino
            print(f"Distance received: {distance}")


            if distance:
                try:
                    distance = float(distance)
                    print(f"Distance: {distance} cm")

                    # Adjust robot behavior based on distance
                    if distance <= GRIP_DISTANCE:
                        send_command("STOP")  # Stop the robot
                        # send_command("GRAB")  # Trigger gripping mechanism
                        break  # Stop loop after gripping
                    elif distance <= SLOW_DISTANCE:
                        send_command("SLOW FORWARD")  # Slow forward
                    else:
                        if abs(error) < 10:  # If the object is near the center
                            send_command("CENTER")  # Move forward
                        elif correction > 0:
                            send_command("RIGHT")  # TuRern right
                        else:
                            send_command("LEFT")  # Turn left

                except ValueError:
                    print("Error: Invalid distance value received.")
        # Kiểm tra phản hồi từ Arduino
        if arduino.in_waiting > 0:
            response = arduino.readline().decode().strip()
            if response == "GRABBED":
                print("Bottle grabbed successfully!")
                send_command("STOP")
                break
  # Chờ trong thời gian ngắn để robot thực hiện hành động
        # Hiển thị hình ảnh
        # cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Nhấn 'q' để thoát
            send_command("STOP")  # Stop the robot
            break

except KeyboardInterrupt:
    print("Terminated by user")

finally:
    camera.release()
    cv2.destroyAllWindows()
