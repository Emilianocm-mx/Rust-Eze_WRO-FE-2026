# sensors/ultrasonic.py
# Módulo para sensor ultrasónico HC-SR04

from machine import Pin, time_pulse_us
import time

class Ultrasonic:
    """
    Driver para HC-SR04.
    Mide distancia en centímetros.
    """
    SPEED_OF_SOUND_CM_US = 0.0343  # cm por microsegundo
    MAX_DISTANCE_CM = 400
    TIMEOUT_US = 30000  # 30ms timeout

    def __init__(self, trig_pin: int, echo_pin: int):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.off()

    def read_cm(self) -> float:
        """
        Devuelve la distancia en cm.
        Retorna MAX_DISTANCE_CM si no hay objeto detectado.
        """
        # Pulso de disparo
        self.trig.off()
        time.sleep_us(2)
        self.trig.on()
        time.sleep_us(10)
        self.trig.off()

        # Medir eco
        duration = time_pulse_us(self.echo, 1, self.TIMEOUT_US)

        if duration < 0:
            return self.MAX_DISTANCE_CM

        distance = (duration * self.SPEED_OF_SOUND_CM_US) / 2
        return min(distance, self.MAX_DISTANCE_CM)

    def read_avg(self, samples: int = 3) -> float:
        """Promedio de varias lecturas para mayor estabilidad."""
        readings = []
        for _ in range(samples):
            d = self.read_cm()
            if d < self.MAX_DISTANCE_CM:
                readings.append(d)
            time.sleep_ms(10)

        if not readings:
            return self.MAX_DISTANCE_CM
        return sum(readings) / len(readings)
