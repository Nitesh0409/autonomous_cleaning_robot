import cv2
import numpy as np

class CameraNode:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        self.lower_green = np.array([35, 100, 100])
        self.upper_green = np.array([85, 255, 255])

    def get_track_target(self):
        """Returns (error_x, area) if target is found, else (None, 0)"""
        ret, frame = self.cap.read()
        if not ret:
            return None, 0

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 300:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    error_x = cx - 160 # 160 is center of 320px width
                    return error_x, area
        
        return None, 0

    def release(self):
        self.cap.release()
