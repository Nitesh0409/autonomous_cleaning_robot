import cv2
import numpy as np

try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False

class ObstacleDetector:
    def __init__(self, model_variant="yolov8n"):
        """
        Initializes YOLOv8 for general object detection.
        model_variant: 'yolov8n' (nano), 'yolov8s' (small). Nano is best for robotics.
        """
        self.has_yolo = HAS_YOLO
        if self.has_yolo:
            print(f"[PERCEPTION] Loading YOLO model: {model_variant}...")
            self.model = YOLO(f"{model_variant}.pt")
        else:
            print("[WARNING] 'ultralytics' library not found. Falling back to simple detection.")
            print("[HINT] Run: pip install ultralytics")

    def detect(self, frame):
        """
        Performs inference and returns detected objects as a list of dicts.
        """
        if not self.has_yolo or frame is None:
            return {"objects": []}

        # Run YOLOv8 Inference
        results = self.model(frame, verbose=False)[0]
        
        detections = []
        for box in results.boxes:
            # Get Box Coordinates, Class Name, and Confidence
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = box.conf[0].item()
            cls_id = int(box.cls[0].item())
            name = self.model.names[cls_id]

            # Represent as a common structure for planning
            detections.append({
                "class": name,
                "confidence": conf,
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)]
            })

        return {"objects": detections}

    def annotate_frame(self, frame, detection_results):
        """Standard Bounding Box visualization"""
        res = frame.copy()
        
        if not self.has_yolo:
            cv2.putText(res, "YOLO NOT INSTALLED (pip install ultralytics)", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return res

        for obj in detection_results["objects"]:
            b = obj["bbox"]
            label = f"{obj['class']} {obj['confidence']:.2f}"
            
            # Color coding (e.g., person is blue, others are green)
            color = (0, 255, 0) if obj["class"] != "person" else (255, 100, 0)
            
            cv2.rectangle(res, (b[0], b[1]), (b[2], b[3]), color, 2)
            cv2.putText(res, label, (b[0], b[1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return res
