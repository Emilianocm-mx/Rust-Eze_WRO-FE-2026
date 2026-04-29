# sensors/infrared.py
# Array de sensores infrarrojos para detección de líneas/bordes

from machine import Pin, ADC

class InfraredArray:
    """
    3 sensores IR analógicos.
    Útil para detectar las líneas naranjas/azules del campo
    y las líneas punteadas de las zonas.
    """
    THRESHOLD = 2000  # Valor ADC umbral (ajustar en calibración)

    def __init__(self, pin_left: int, pin_center: int, pin_right: int):
        self.left   = ADC(Pin(pin_left))
        self.center = ADC(Pin(pin_center))
        self.right  = ADC(Pin(pin_right))

        # Rango completo (0-4095 en ESP32)
        for sensor in [self.left, self.center, self.right]:
            sensor.atten(ADC.ATTN_11DB)

    def read_raw(self) -> tuple:
        """Devuelve los valores crudos (izq, centro, der)."""
        return (
            self.left.read(),
            self.center.read(),
            self.right.read()
        )

    def read_digital(self) -> tuple:
        """
        Devuelve True si el sensor detecta línea (valor sobre umbral).
        (izq, centro, der)
        """
        l, c, r = self.read_raw()
        return (
            l > self.THRESHOLD,
            c > self.THRESHOLD,
            r > self.THRESHOLD
        )

    def detected_line(self) -> bool:
        """True si cualquier sensor detecta una línea."""
        return any(self.read_digital())

    def position_error(self) -> float:
        """
        Error de posición lateral respecto a la línea central.
        Útil para control PID de seguimiento de carril.
        Rango: -1.0 (muy izquierda) a +1.0 (muy derecha)
        """
        l, c, r = self.read_raw()
        total = l + c + r
        if total == 0:
            return 0.0
        # Centro de masa normalizado
        weighted = (-1.0 * l + 0.0 * c + 1.0 * r) / total
        return weighted

    def calibrate(self, threshold: int):
        """Actualiza el umbral de detección."""
        self.THRESHOLD = threshold
