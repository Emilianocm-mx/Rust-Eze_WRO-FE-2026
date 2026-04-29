# =============================================================
# WRO 2026 - Futuros Ingenieros
# Robot: Carro Autónomo con ESP32
# Equipo: [Nombre del equipo]
# =============================================================

from machine import Pin, PWM, I2C
import time
from sensors.ultrasonic import Ultrasonic
from sensors.infrared import InfraredArray
from sensors.imu import IMU
from camera.color_detector import ColorDetector
from control.motor_controller import MotorController
from control.steering import Steering
from strategy.open_challenge import OpenChallenge
from strategy.obstacle_challenge import ObstacleChallenge

# ── Pines ────────────────────────────────────────────────────
# Ultrasónicos
PIN_TRIG_LEFT   = 5
PIN_ECHO_LEFT   = 18
PIN_TRIG_RIGHT  = 19
PIN_ECHO_RIGHT  = 21
PIN_TRIG_FRONT  = 22
PIN_ECHO_FRONT  = 23

# Infrarrojos (array de 3 sensores)
PIN_IR_LEFT     = 34
PIN_IR_CENTER   = 35
PIN_IR_RIGHT    = 32

# Motor tracción
PIN_MOTOR_PWM   = 25
PIN_MOTOR_DIR   = 26

# Servo dirección
PIN_SERVO       = 27

# IMU (I2C)
PIN_SDA         = 16
PIN_SCL         = 17

# Botones
PIN_START       = 0   # Botón de inicio (Boot button del ESP32)
PIN_POWER       = 2   # LED indicador de encendido

# ── Inicialización ────────────────────────────────────────────
def init_hardware():
    # Indicador de encendido
    led = Pin(PIN_POWER, Pin.OUT)
    led.on()

    # Sensores
    us_left  = Ultrasonic(PIN_TRIG_LEFT,  PIN_ECHO_LEFT)
    us_right = Ultrasonic(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT)
    us_front = Ultrasonic(PIN_TRIG_FRONT, PIN_ECHO_FRONT)

    ir = InfraredArray(PIN_IR_LEFT, PIN_IR_CENTER, PIN_IR_RIGHT)

    i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400000)
    imu = IMU(i2c)

    camera = ColorDetector()  # ESP32-CAM via UART

    # Actuadores
    motor   = MotorController(PIN_MOTOR_PWM, PIN_MOTOR_DIR)
    servo   = Steering(PIN_SERVO)

    return us_left, us_right, us_front, ir, imu, camera, motor, servo

# ── Selección de reto ─────────────────────────────────────────
def select_challenge(start_btn):
    """
    Mantén presionado el botón al encender para Reto con Obstáculos.
    Suéltalo para Reto Abierto.
    """
    print("Esperando botón de inicio...")
    print("  - Presión corta: Reto Abierto")
    print("  - Presión larga (3s): Reto con Obstáculos")

    # Esperar a que se presione
    while start_btn.value() == 1:
        time.sleep(0.1)

    press_start = time.ticks_ms()
    while start_btn.value() == 0:
        time.sleep(0.1)

    duration = time.ticks_diff(time.ticks_ms(), press_start)
    return "obstacle" if duration >= 3000 else "open"

# ── Punto de entrada ──────────────────────────────────────────
def main():
    print("=== WRO 2026 - Iniciando sistema ===")

    # Inicializar hardware
    us_left, us_right, us_front, ir, imu, camera, motor, servo = init_hardware()

    start_btn = Pin(PIN_START, Pin.IN, Pin.PULL_UP)

    # Seleccionar reto
    challenge_type = select_challenge(start_btn)
    print(f"Reto seleccionado: {challenge_type}")

    # Ejecutar estrategia
    if challenge_type == "open":
        strategy = OpenChallenge(us_left, us_right, us_front, ir, imu, motor, servo)
    else:
        strategy = ObstacleChallenge(us_left, us_right, us_front, ir, imu, camera, motor, servo)

    print("¡Iniciando reto!")
    strategy.run()
    print("Reto finalizado.")

main()
