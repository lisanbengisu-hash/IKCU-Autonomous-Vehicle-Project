import threading
import time
import cv2
from config import YOLO_MODEL_PATH, YOLO_CONFIDENCE, YOLO_IMAGE_SIZE, YOLO_ALLOWED_CLASSES

class YOLODetector:
    def __init__(self):
        self.model = None
        self.detections = []
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def load(self):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise RuntimeError("ultralytics kurulu degil: pip install ultralytics") from e
        print("YOLO modeli yukleniyor:", YOLO_MODEL_PATH)
        self.model = YOLO(YOLO_MODEL_PATH)
        print("YOLO hazir.")

    def start(self, frame_provider):
        if self.model is None:
            self.load()
        self.running = True
        self.thread = threading.Thread(target=self._loop, args=(frame_provider,), daemon=True)
        self.thread.start()

    def _loop(self, frame_provider):
        while self.running:
            frame = frame_provider()
            if frame is None:
                time.sleep(0.01)
                continue
            try:
                results = self.model.predict(
                    frame, conf=YOLO_CONFIDENCE, imgsz=YOLO_IMAGE_SIZE,
                    classes=YOLO_ALLOWED_CLASSES, verbose=False
                )
                annotated = frame.copy()
                found = []
                if results:
                    r = results[0]
                    if r.boxes is not None:
                        for box in r.boxes:
                            cid = int(box.cls[0].item())
                            conf = float(box.conf[0].item())
                            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                            label = r.names.get(cid, str(cid))
                            found.append({
                                "class_id": cid, "label": label, "confidence": conf,
                                "bbox": (x1, y1, x2, y2),
                                "center": ((x1+x2)//2, (y1+y2)//2)
                            })
                            cv2.rectangle(annotated, (x1,y1), (x2,y2), (255,255,255), 2)
                            cv2.putText(annotated, f"{label} {conf:.2f}",
                                        (x1, max(20,y1-8)), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.55, (255,255,255), 2, cv2.LINE_AA)
                with self.lock:
                    self.detections = found
                    self.frame = annotated
            except Exception as e:
                print("YOLO inference hatasi:", repr(e))
                time.sleep(0.1)

    def get_detections(self):
        with self.lock:
            return list(self.detections)

    def get_annotated_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
