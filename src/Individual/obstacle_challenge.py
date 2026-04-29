# strategy/obstacle_challenge.py
# Estrategia para el Reto con Obstáculos WRO 2026

import time

class ObstacleChallenge:
    """
    Estrategia Reto con Obstáculos.

    Estados principales:
    - LANE_FOLLOW: seguimiento de muros en recta
    - DODGE:       esquivar pilar detectado por cámara
    - TURNING:     ejecutando curva de 90°
    - PARKING:     maniobra de estacionamiento paralelo
    - DONE:        reto completado
    
    Regla WRO: 
      Pilar ROJO  → pasar por la derecha (esquivar a la izquierda)
      Pilar VERDE → pasar por la izquierda (esquivar a la derecha)
    """

    # Parámetros de control
    BASE_SPEED      = 45
    DODGE_SPEED     = 35
    TURN_SPEED      = 30
    PARK_SPEED      = 25
    WALL_TARGET_CM  = 30
    FRONT_BRAKE_CM  = 30
    PILLAR_NEAR_PCT = 15   # % del frame para iniciar esquive
    KP_WALL         = 0.8
    KP_DODGE        = 1.2
    SECTIONS_TOTAL  = 24   # 3 vueltas × 8 secciones

    # Estados
    STATE_LANE_FOLLOW = "LANE_FOLLOW"
    STATE_DODGE       = "DODGE"
    STATE_TURNING     = "TURNING"
    STATE_PARKING     = "PARKING"
    STATE_DONE        = "DONE"

    def __init__(self, us_left, us_right, us_front, ir, imu, camera, motor, servo):
        self.us_left  = us_left
        self.us_right = us_right
        self.us_front = us_front
        self.ir       = ir
        self.imu      = imu
        self.camera   = camera
        self.motor    = motor
        self.servo    = servo

        self.state         = self.STATE_LANE_FOLLOW
        self.sections_done = 0
        self.laps_done     = 0

    # ── Seguimiento de muros ──────────────────────────────────
    def _wall_error(self) -> float:
        left  = self.us_left.read_cm()
        right = self.us_right.read_cm()
        return left - right

    # ── Detección de curva ────────────────────────────────────
    def _front_clear(self) -> bool:
        return self.us_front.read_cm() > self.FRONT_BRAKE_CM

    # ── Esquive de pilar ──────────────────────────────────────
    def _dodge_angle(self) -> float:
        """
        Calcula el ángulo de esquive según color y posición del pilar.
        Pilar rojo: desviarse a la izquierda (ángulo negativo)
        Pilar verde: desviarse a la derecha (ángulo positivo)
        """
        color = self.camera.pillar_color()
        x     = self.camera.pillar_x()   # 0=izq, 319=der
        center = 160

        offset = (x - center) / center  # -1 a +1

        if color == "red":
            # Pasar por la derecha del pilar → alejarse hacia la izquierda
            return -15 + offset * 5
        elif color == "green":
            # Pasar por la izquierda del pilar → alejarse hacia la derecha
            return 15 + offset * 5
        return 0.0

    # ── Curva ─────────────────────────────────────────────────
    def _turn(self):
        print(f"[Obs] Curva. Sección {self.sections_done}")
        self.motor.set_speed(self.TURN_SPEED)
        self.servo.right(25)
        self.imu.wait_for_turn(target_degrees=85, timeout_ms=3000)
        self.servo.center()
        self.sections_done += 1

        if self.sections_done % 8 == 0:
            self.laps_done += 1
            print(f"[Obs] Vuelta {self.laps_done} completada.")

    # ── Estacionamiento ───────────────────────────────────────
    def _park(self):
        """
        Maniobra de estacionamiento paralelo en cajón magenta.
        La cámara detecta los delimitadores magenta.
        Estrategia: alinearse con el cajón y entrar en reversa.
        """
        print("[Obs] Iniciando estacionamiento...")

        # 1. Avanzar lentamente buscando el cajón
        self.motor.set_speed(self.PARK_SPEED)
        self.servo.center()
        time.sleep(0.5)

        # 2. Alinearse lateralmente con el cajón
        # (La cámara detecta magenta y ajusta la posición)
        align_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), align_time) < 2000:
            self.camera.update()
            # Aquí se puede agregar lógica de alineación con magenta
            time.sleep_ms(50)

        # 3. Entrar en reversa al cajón
        self.servo.center()
        self.motor.set_speed(-self.PARK_SPEED)
        time.sleep(1.0)

        # 4. Detener
        self.motor.brake()
        self.servo.center()
        print("[Obs] Estacionado.")

    # ── Loop principal ────────────────────────────────────────
    def run(self):
        print(f"[Obs] Iniciando. Meta: {self.SECTIONS_TOTAL} secciones.")
        self.imu.reset_yaw()
        self.motor.set_speed(self.BASE_SPEED)

        while self.state != self.STATE_DONE:
            self.imu.update()
            self.camera.update()

            if self.state == self.STATE_LANE_FOLLOW:
                # Verificar si hay pilar cercano
                if self.camera.sees_pillar() and \
                   self.camera.pillar_size() >= self.PILLAR_NEAR_PCT:
                    self.state = self.STATE_DODGE
                    continue

                # Verificar curva
                if not self._front_clear():
                    self.state = self.STATE_TURNING
                    continue

                # Seguimiento normal de muros
                error = self._wall_error()
                self.servo.set_angle(self.KP_WALL * error)
                self.motor.set_speed(self.BASE_SPEED)

            elif self.state == self.STATE_DODGE:
                # Esquivar el pilar
                if not self.camera.sees_pillar() or \
                   self.camera.pillar_size() < 5:
                    # Pilar superado, volver a seguimiento
                    self.servo.center()
                    self.state = self.STATE_LANE_FOLLOW
                    continue

                angle = self._dodge_angle()
                self.servo.set_angle(angle)
                self.motor.set_speed(self.DODGE_SPEED)

            elif self.state == self.STATE_TURNING:
                self._turn()
                self.state = self.STATE_LANE_FOLLOW

                # ¿Completamos las 3 vueltas?
                if self.sections_done >= self.SECTIONS_TOTAL:
                    self.state = self.STATE_PARKING

            elif self.state == self.STATE_PARKING:
                self._park()
                self.state = self.STATE_DONE

            time.sleep_ms(20)

        print("[Obs] Reto completado.")
        self.motor.stop()
