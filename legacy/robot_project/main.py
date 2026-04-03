import cv2
import time
from sensors.camera_node import CameraNode
from perception.obstacle_detection import ObstacleDetector

def main():
    """Main loop for General Perception (Task 1: YOLO Object Detection)"""
    # 1. Initialize Components
    print("--- [ROBOT PERCEPTION INITIALIZING] ---")
    camera = CameraNode(0) 
    detector = ObstacleDetector(model_variant="yolov8n") # Nano model for speed

    print("--- [OBJECT DETECTION MODE: ACTIVE] ---")
    print("Detecting general objects (People, Chairs, obstacles)...")
    print("Press 'q' to quit.")

    # 2. Perception Loop
    while True:
        ret, frame = camera.cap.read()
        if not ret: 
            print("Failed to capture image")
            break

        # I - Detection (YOLO Inference)
        results = detector.detect(frame)
        
        # II - Visualization (Bounding Boxes)
        annotated_frame = detector.annotate_frame(frame, results)
        cv2.imshow('Robot Perception - YOLO Object Detection', annotated_frame)

        # III - Telemetry
        num_objects = len(results["objects"])
        if num_objects > 0:
            # Group by class
            classes = [o["class"] for o in results["objects"]]
            unique_classes = set(classes)
            summary = ", ".join([f"{classes.count(c)} {c}(s)" for c in unique_classes])
            print(f"[DETECTED]: {summary}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 3. Cleanup
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
