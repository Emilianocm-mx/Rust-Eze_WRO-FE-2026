"""LIDAR / ultrasonic sensor interface."""

class Lidar:
    def __init__(self, config):
        self.port = config.get("port", "/dev/ttyUSB0")

    def scan(self) -> dict:
        """Retorna distancias en mm para cada sector {front, left, right}."""
        return {"front": 9999, "left": 9999, "right": 9999}
