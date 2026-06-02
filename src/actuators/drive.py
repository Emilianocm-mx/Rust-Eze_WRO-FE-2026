"""Drive controller — motor y servo de dirección."""

class Drive:
    def __init__(self, config):
        self.speed = config.get("default_speed", 50)

    def update(self, frame, scan):
        pass

    def stop(self):
        print("Motores detenidos.")
