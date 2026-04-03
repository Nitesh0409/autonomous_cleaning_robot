import cv2
import numpy as np

def test_mastermind_cv():
    # Attempt to open the first available webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # HSV thresholds for "Neon green trash"
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    print("Mastermind Simulator started. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Resize for consistent logic with ESP32-CAM (320x240)
        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Color masking
        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        linear_x = 0.0
        angular_z = 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: # Min threshold to ignore noise
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Logic: center width is 160
                    error_x = cx - 160
                    angular_z = -float(error_x) / 100.0
                    linear_x = 0.5
                    
                    # Visual feedback
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(frame, f"CHASING! Vel: {linear_x}, Ang: {angular_z:.2f}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        else:
            # Idle rotation
            linear_x = 0.0
            angular_z = 0.3
            cv2.putText(frame, "SCANNING...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Show the result
        cv2.imshow('Mastermind CV Test (Webcam)', frame)
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_mastermind_cv()
