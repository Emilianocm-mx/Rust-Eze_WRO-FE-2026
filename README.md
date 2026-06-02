Engineering materials
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2026.

## Content

* `t-photos` contains 4 photos of the team (One individual photo of each member and one team photo).
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom).
* `video` contains the video.md file with the link to the video where driving demonstration exists.
* `src` contains code of control software for all components which were programmed.
* `models` files for models used by 3D printers.

## Introduction

_Code Modules and Structure
The software for this autonomous vehicle is designed using a modular architecture to separate high-level decision-making from low-level hardware control. It consists of the following main modules:

Vision Module (ESP32-CAM): Developed in MicroPython/C++, this module handles real-time image processing. It identifies the color signatures of the pillars (Red and Green) and the parking area (Magenta). It communicates the detection results to the main controller via UART Serial.

Navigation & Strategy Module (Main ESP32): The core logic that interprets sensor data. It manages the "Open Challenge" (wall-following) and the "Obstacle Challenge" (avoiding pillars) based on the input from the Vision Module and Ultrasonic sensors.

Hardware Abstraction Layer (HAL): A set of functions that control the electromechanical components directly, such as the PWM signals for the steering servo and the H-Bridge (TB6612FNG) for the traction motor.

Kinematics Module: Specifically designed to handle the Ackerman steering geometry, ensuring the servo angles correspond correctly to the vehicle's turning radius.

Relation to Electromechanical Components
The code interacts with the vehicle’s hardware as follows:

Steering: A Servo Motor on the front axle is controlled via PWM. The code calculates the necessary angle to either stay centered between walls or overtake an obstacle.

Propulsion: A Pololu DC Motor is driven by the TB6612FNG controller. The code uses PWM to maintain a constant speed and digital pins to manage the driving direction (Forward/Reverse).

Spatial Awareness: Ultrasonic sensors (HC-SR04) provide distance data to the side walls, allowing the code to perform PID-based lane centering.

Orientation: An IMU sensor (MPU6050) is used to detect 90-degree turns and maintain a straight heading, which is crucial for counting laps and executing the parallel parking maneuver.

Build, Compile, and Upload Process
The development environment used for this project is Thonny IDE (or VS Code with the Pymakr extension) using MicroPython firmware.

Firmware Installation: The ESP32 and ESP32-CAM controllers are first flashed with the latest MicroPython stable firmware using the esptool.py command-line utility.

Upload Process:

Connect the ESP32 to the computer via USB.

The scripts (main.py, boot.py, and library files) are uploaded directly to the internal flash memory of the microcontroller.

Execution: Upon power-up, the boot.py initializes the hardware, and the main.py enters a standby mode, waiting for the physical "Start" button to be pressed, as required by the WRO 2026 regulations.
