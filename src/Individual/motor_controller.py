# control/motor_controller.py
from machine import Pin, PWM

class MotorController:
    """
    Control de motor DC de tracción via PWM.
    Compatible con driver L298N / L9110 / Cytron.
    """
    FREQ = 1000  # Hz

    def __init__(self, pwm_pin: int, dir_pin: int):
        self.pwm = PWM(Pin(pwm_pin), freq=self.FREQ, duty=0)
        self.dir = Pin(dir_pin, Pin.OUT)
        self._speed = 0

    def set_speed(self, speed: int):
        """
        Velocidad de -100 a 100.
        Positivo = adelante, negativo = atrás, 0 = parar.
        """
        speed = max(-100, min(100, speed))
        self._speed = speed

        if speed >= 0:
            self.dir.off()
            duty = int(speed / 100 * 1023)
        else:
            self.dir.on()
            duty = int(abs(speed) / 100 * 1023)

        self.pwm.duty(duty)

    def stop(self):
        self.set_speed(0)

    def brake(self):
        """Freno activo."""
        self.pwm.duty(0)
        self.dir.on()

    @property
    def speed(self):
        return self._speed


# control/steering.py
from machine import Pin, PWM
import time

class Steering:
    """
    Control de servo de dirección.
    Centro = recto, izquierda/derecha en grados.
    """
    FREQ       = 50    # Hz (estándar servo)
    CENTER_US  = 1500  # microsegundos = recto
    MIN_US     = 1000  # máximo izquierda
    MAX_US     = 2000  # máximo derecha
    MAX_ANGLE  = 30    # grados máximos de giro

    def __init__(self, pin: int):
        self.pwm = PWM(Pin(pin), freq=self.FREQ)
        self.set_angle(0)  # centrar al inicio

    def _us_to_duty(self, us: int) -> int:
        """Convierte microsegundos a valor de duty (0-1023)."""
        period_us = 1_000_000 // self.FREQ
        return int(us / period_us * 1023)

    def set_angle(self, angle: float):
        """
        Ángulo en grados: -MAX_ANGLE (izq) a +MAX_ANGLE (der).
        0 = recto.
        """
        angle = max(-self.MAX_ANGLE, min(self.MAX_ANGLE, angle))
        us = int(self.CENTER_US + (angle / self.MAX_ANGLE) * (self.MAX_US - self.CENTER_US))
        self.pwm.duty(self._us_to_duty(us))

    def center(self):
        self.set_angle(0)

    def left(self, degrees: float = 20):
        self.set_angle(-abs(degrees))

    def right(self, degrees: float = 20):
        self.set_angle(abs(degrees))
