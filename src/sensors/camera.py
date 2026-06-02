"""Camera module — wraps OpenCV capture and preprocessing."""
import cv2

class Camera:
    def __init__(self, config):
        self.cap = cv2.VideoCapture(config.get("index", 0))
        self.width = config.get("width", 640)
        self.height = config.get("height", 480)

    def capture(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("No se pudo capturar frame de la cámara.")
        return cv2.resize(frame, (self.width, self.height))

    def release(self):
        self.cap.release()
