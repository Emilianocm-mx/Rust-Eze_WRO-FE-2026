# sensors/imu.py
# Driver para IMU (MPU-6050 o similar por I2C)
# Usado para mantener rumbo recto y contar giros de 90°

from machine import I2C
import struct
import time

class IMU:
    """
    Driver básico para MPU-6050.
    Provee:
      - Ángulo de rumbo (yaw) acumulado via giroscopio Z
      - Detección de giros de 90° para contar secciones/vueltas
    """
    MPU6050_ADDR  = 0x68
    PWR_MGMT_1    = 0x6B
    GYRO_CONFIG   = 0x1B
    GYRO_ZOUT_H   = 0x47
    GYRO_SCALE    = 131.0  # LSB/(°/s) para rango ±250°/s

    def __init__(self, i2c: I2C):
        self.i2c = i2c
        self.yaw = 0.0
        self._last_time = time.ticks_ms()

        self._init_device()

    def _init_device(self):
        # Despertar el MPU6050
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.PWR_MGMT_1, b'\x00')
        time.sleep_ms(100)
        # Rango del giroscopio: ±250°/s
        self.i2c.writeto_mem(self.MPU6050_ADDR, self.GYRO_CONFIG, b'\x00')

    def _read_gyro_z(self) -> float:
        """Lee la velocidad angular en Z (°/s)."""
        data = self.i2c.readfrom_mem(self.MPU6050_ADDR, self.GYRO_ZOUT_H, 2)
        raw = struct.unpack('>h', data)[0]
        return raw / self.GYRO_SCALE

    def update(self):
        """
        Integra el giroscopio para actualizar el ángulo acumulado (yaw).
        Llamar en cada ciclo del loop principal.
        """
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time) / 1000.0  # segundos
        self._last_time = now

        gz = self._read_gyro_z()
        self.yaw += gz * dt

    def get_yaw(self) -> float:
        """Devuelve el ángulo acumulado en grados."""
        return self.yaw

    def reset_yaw(self):
        """Reinicia el ángulo de referencia."""
        self.yaw = 0.0

    def is_turning(self, threshold_dps: float = 20.0) -> bool:
        """True si el robot está girando activamente."""
        return abs(self._read_gyro_z()) > threshold_dps

    def wait_for_turn(self, target_degrees: float, timeout_ms: int = 3000):
        """
        Espera hasta que el robot haya girado `target_degrees`.
        Útil para detectar el paso por una curva de 90°.
        """
        self.reset_yaw()
        start = time.ticks_ms()
        while abs(self.yaw) < abs(target_degrees):
            self.update()
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                break
            time.sleep_ms(10)
