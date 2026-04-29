# camera/color_detector.py
# Detección de pilares rojo y verde usando ESP32-CAM
# La ESP32-CAM corre su propio firmware y envía resultados por UART

from machine import UART
import json

class ColorDetector:
    """
    Interfaz con ESP32-CAM via UART.
    
    El firmware de la cámara procesa frames y envía JSON:
    {"color": "red"|"green"|"none", "x": 0-319, "size": 0-100}
    
    color: color detectado
    x:     posición horizontal del pilar (0=izquierda, 319=derecha)
    size:  tamaño aproximado en % del frame (0=lejos, 100=muy cerca)
    """
    UART_ID   = 1
    BAUD_RATE = 115200
    PIN_TX    = 10
    PIN_RX    = 9

    # Colores RGB objetivo (según reglamento WRO 2026)
    RED_RGB   = (238, 39, 55)
    GREEN_RGB = (68, 214, 44)

    def __init__(self):
        self.uart = UART(self.UART_ID,
                         baudrate=self.BAUD_RATE,
                         tx=self.PIN_TX,
                         rx=self.PIN_RX)
        self._last = {"color": "none", "x": 160, "size": 0}

    def update(self):
        """
        Lee el último frame analizado por la cámara.
        Llamar en cada ciclo del loop.
        """
        if self.uart.any():
            try:
                line = self.uart.readline().decode().strip()
                data = json.loads(line)
                self._last = data
            except Exception:
                pass  # Ignorar frames corruptos

    def get_detection(self) -> dict:
        """Devuelve el último resultado de detección."""
        return self._last

    def sees_pillar(self) -> bool:
        """True si hay un pilar visible."""
        return self._last["color"] in ("red", "green")

    def pillar_color(self) -> str:
        """'red', 'green', o 'none'."""
        return self._last["color"]

    def pillar_x(self) -> int:
        """Posición X del pilar (0=izq, 319=der, 160=centro)."""
        return self._last.get("x", 160)

    def pillar_size(self) -> int:
        """Tamaño del pilar en % del frame. Mayor = más cerca."""
        return self._last.get("size", 0)

    def should_dodge_left(self) -> bool:
        """
        True si el pilar requiere esquivar a la izquierda.
        Pilar ROJO → pasar por la derecha → esquivar a la izquierda
        """
        return self._last["color"] == "red"

    def should_dodge_right(self) -> bool:
        """
        True si el pilar requiere esquivar a la derecha.
        Pilar VERDE → pasar por la izquierda → esquivar a la derecha
        """
        return self._last["color"] == "green"
