# strategy/open_challenge.py
# Estrategia para el Reto Abierto WRO 2026
# El robot da 3 vueltas completas evitando muros y se detiene en la sección de arranque.

import time

class OpenChallenge:
    """
    Estrategia Reto Abierto.

    Lógica principal:
    1. Mantener distancia central entre muros usando los ultrasónicos laterales.
    2. Detectar curvas con el ultrasónico frontal + IMU.
    3. Contar secciones con el IMU (cada 90° = 1 curva = avance de sección).
    4. Tras 24 secciones (3 vueltas × 8 secciones), detener en zona de arranque.
    """

    # Parámetros de control (ajustar en pruebas)
    BASE_SPEED      = 50    # velocidad crucero (0-100)
    TURN_SPEED      = 35    # velocidad en curva
    WALL_TARGET_CM  = 30    # distancia objetivo a cada muro lateral
    FRONT_BRAKE_CM  = 25    # frenar si hay muro a esta distancia al frente
    KP              = 0.8   # ganancia proporcional del controlador de dirección
    SECTIONS_TOTAL  = 24    # 3 vueltas × 8 secciones

    def __init__(self, us_left, us_right, us_front, ir, imu, motor, servo):
        self.us_left  = us_left
        self.us_right = us_right
        self.us_front = us_front
        self.ir       = ir
        self.imu      = imu
        self.motor    = motor
        self.servo    = servo

        self.sections_done = 0
        self.in_turn       = False

    def _wall_follow_angle(self) -> float:
        """
        Control P para mantenerse centrado entre muros.
        Error positivo → girar a la derecha.
        """
        left  = self.us_left.read_cm()
        right = self.us_right.read_cm()
        error = left - right  # positivo = más cerca de la izquierda
        return self.KP * error

    def _detect_turn(self) -> bool:
        """Detecta entrada a curva: muro frontal cerca."""
        return self.us_front.read_cm() < self.FRONT_BRAKE_CM

    def _execute_turn(self):
        """Gira 90° usando el IMU para medir el ángulo."""
        self.motor.set_speed(self.TURN_SPEED)
        self.servo.right(25)  # ajustar dirección según sentido del circuito
        self.imu.wait_for_turn(target_degrees=85, timeout_ms=3000)
        self.servo.center()
        self.sections_done += 1
        self.in_turn = False

    def _stop_in_start_section(self):
        """Detener el robot en la sección de arranque."""
        self.motor.set_speed(20)
        time.sleep(0.5)
        self.motor.brake()
        self.servo.center()

    def run(self):
        """Loop principal del Reto Abierto."""
        print(f"[Open] Iniciando. Meta: {self.SECTIONS_TOTAL} secciones.")
        self.imu.reset_yaw()
        self.motor.set_speed(self.BASE_SPEED)

        while self.sections_done < self.SECTIONS_TOTAL:
            self.imu.update()

            if self._detect_turn() and not self.in_turn:
                # Entrada a curva
                self.in_turn = True
                print(f"[Open] Curva detectada. Secciones: {self.sections_done}")
                self._execute_turn()
            else:
                # Seguimiento de muros en recta
                angle = self._wall_follow_angle()
                self.servo.set_angle(angle)
                self.motor.set_speed(self.BASE_SPEED)

            time.sleep_ms(20)

        # Completadas 3 vueltas → detener en sección de arranque
        print("[Open] 3 vueltas completadas. Deteniendo.")
        self._stop_in_start_section()
