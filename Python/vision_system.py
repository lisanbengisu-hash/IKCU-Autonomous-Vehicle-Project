import threading, time, cv2
from camera import RGBCamera
from yolo_detector import YOLODetector

class VisionSystem:
    WINDOW_NAME="IKCU Shuttle - YOLO"
    def __init__(self, world, vehicle):
        self.camera=RGBCamera(world, vehicle); self.detector=YOLODetector()
        self.running=False; self.thread=None
    def start(self):
        self.camera.start(); self.detector.start(self.camera.get_frame)
        self.running=True
        self.thread=threading.Thread(target=self._display_loop, daemon=True); self.thread.start()
        print("VISION SYSTEM HAZIR - surus loopundan bagimsiz.")
    def _display_loop(self):
        while self.running:
            frame=self.detector.get_annotated_frame()
            if frame is not None:
                try:
                    cv2.imshow(self.WINDOW_NAME, frame); cv2.waitKey(1)
                except Exception: pass
            time.sleep(0.03)
    def get_detections(self):
        return self.detector.get_detections()
    def destroy(self):
        self.running=False
        if self.thread: self.thread.join(timeout=1.0)
        try: self.detector.stop()
        except Exception: pass
        try: self.camera.destroy()
        except Exception: pass
        try: cv2.destroyAllWindows()
        except Exception: pass
