#!/usr/bin/env python3
"""
Rust-eze — WRO 2026 Future Engineers
Main entry point for the autonomous robot.
"""

from sensors.camera import Camera
from sensors.lidar import Lidar
from actuators.drive import Drive
from utils.config import Config


def main():
    config = Config.load("config.yaml")
    camera = Camera(config.camera)
    lidar = Lidar(config.lidar)
    drive = Drive(config.drive)

    print("Rust-eze iniciando secuencia autónoma...")

    try:
        while True:
            frame = camera.capture()
            scan = lidar.scan()
            drive.update(frame, scan)
    except KeyboardInterrupt:
        drive.stop()
        print("Detenido por el usuario.")


if __name__ == "__main__":
    main()
