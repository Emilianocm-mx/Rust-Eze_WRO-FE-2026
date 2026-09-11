# 🤖 Rust-eze — WRO 2026 Future Engineers

![WRO](https://img.shields.io/badge/WRO-2026-blue)
![Category](https://img.shields.io/badge/Category-Future%20Engineers-green)
![Country](https://img.shields.io/badge/Country-Mexico-red)
![Lang](https://img.shields.io/badge/Code-C%2B%2B%20%7C%20Python-yellow)

Official repository of team **Rust-eze** for the WRO 2026 Future Engineers category. Here you will find the source code, technical documentation, hardware schematics, and multimedia material of our autonomous robot.

---

## 👥 Team

| Name | Role |
|------|------|
| Emi | Mechanical |
| Oliver | Electronics |
| Paco | Programmer |
| Leo | Coach |

---

## 🏗️ System Architecture

```

                    ┌──────────────────────┐
                    │     XIAO ESP32-C6    │
                    │   Main Controller    │
                    └──────────┬───────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         ▼                     ▼                     ▼
   ┌───────────┐         ┌───────────┐         ┌───────────┐
   │  RPLiDAR  │         │ ESP32-CAM │         │  MG90     │
   │   A1M8    │         │           │         │  Servo    │
   │ Distance  │         │  Color    │         │ Steering  │
   └───────────┘         └───────────┘         └───────────┘
         │                     │
         └──────────┬──────────┘
                    │
                    ▼
             ┌─────────────┐
             │  Navigation │
             │    Logic    │
             └──────┬──────┘
                    │
                    ▼
             ┌─────────────┐
             │ TB6612FNG   │
             │ Motor Driver│
             └──────┬──────┘
                    │
                    ▼
             ┌─────────────┐
             │Pololu 300RPM│
             │ Drive Motor │
             └─────────────┘
```
---
```


## 📁 Repository Structure

```


---

## ⚡ Electrical System

### Power Architecture

```
LiPo Battery 7.4V
       │
   [Switch]
       │
   ┌───┴──────────────────┐
   │                      │
   ▼                      ▼
TB6612FNG              LM2596
Motor Driver          Step-down
  (7.4V)               5V out
   │                      │
   ▼               ┌──────┼──────────┐
Pololu Motor        ▼      ▼          ▼
               ESP32-C6  RPLiDAR  ESP32-CAM
```
---
### Wiring Diagram

![Electronic Scheme](schemes/Electronic_Scheme.jpg)

---

## 🛠️ Components

| Photo | Component | Description |
|:-----------:|-----------|-------------|
| <div align="center"><img src="v-photos/XIAOESP32-C6.jpg" width="250" height="250"></div> | **XIAO ESP32-C6** | • Main microcontroller of the robot <br> • Processes RPLiDAR distance data <br> • Controls steering servo and motor driver <br> • Communicates with ESP32-CAM via UART <br> • Runs navigation and PID control logic |
| <div align="center"><img src="v-photos/RPLiDARA1M8.jpg" width="250" height="250"></div> | **RPLiDAR A1M8** | • 360° laser distance scanner <br> • Primary perception sensor for wall following <br> • Detects corners, open spaces and obstacles <br> • Connected to ESP32-C6 via UART (TX/RX) <br> • Powered at 5V from LM2596 |
| <div align="center"><img src="v-photos/ESP32-CAM.jpg" width="250" height="250"></div> | **ESP32-CAM** | • Handles visual color detection <br> • Identifies red and green traffic sign pillars <br> • Sends color decisions to ESP32-C6 via UART <br> • Used exclusively during the Obstacle Challenge |
| <div align="center"><img src="v-photos/TB6612FNG.jpg" width="250" height="250"></div> | **TB6612FNG Motor Driver** | • Controls Pololu DC motor speed and direction <br> • Accepts PWM + direction signals from ESP32-C6 <br> • Powered directly from LiPo at 7.4V <br> • Protects ESP32 from motor current draw |
| <div align="center"><img src="v-photos/Pololu.jpg" width="250" height="250"></div> | **Pololu DC Motor 300 RPM** | • Rear-wheel drive traction motor <br> • Connected to rear axle through physical gear system <br> • Speed controlled via PWM through TB6612FNG <br> • Single motor drives both rear wheels |
| <div align="center"><img src="v-photos/Servo.jpg" width="250" height="250"></div> | **MG90 Micro Servo** | • Controls front steering mechanism <br> • Physical range: 40° (full left) to 140° (full right) <br> • Center position at 90° = straight ahead <br> • PWM signal from ESP32-C6 GPIO 0 |
| <div align="center"><img src="v-photos/LM2596.jpg" width="250" height="250"></div> | **LM2596 Step-Down Regulator** | • Converts 7.4V LiPo down to stable 5V <br> • Powers all logic components safely <br> • Prevents voltage damage to ESP32 and LiDAR <br> • Adjustable output verified before use |
| <div align="center"><img src="v-photos/LiPo.jpg" width="250" height="250"></div> | **LiPo Battery 7.4V** | • Main power source for the entire system <br> • Directly feeds motor driver at full voltage <br> • Feeds LM2596 for 5V logic rail <br> • Lightweight for weight-sensitive robot design |
| <div align="center"><img src="v-photos/Switch.jpg" width="250" height="250"></div> | **Power Switch** | • Master on/off switch for the entire system <br> • Required by WRO rules (one switch to power on) <br> • Cuts all power from battery before start |
| <div align="center"><img src="v-photos/IMU.jpg" width="250" height="250"></div> | **IMU Sensor** | • Inertial measurement unit <br> • Provides heading and orientation data <br> • Supports navigation during turns <br> • Connected via I2C (SDA/SCL) to ESP32-C6 |

---

### Pin Connections

| Component | ESP32-C6 Pin | Function |
|-----------|-------------|----------|
| LiDAR RX | GPIO 19 | Data reception from LiDAR |
| LiDAR TX | GPIO 16 | Data transmission to LiDAR |
| Servo MG90 | GPIO 0 | Steering PWM signal |
| PWMA | GPIO 22 | Motor speed (PWM) |
| AIN1 | GPIO 2 | Motor direction A |
| AIN2 | GPIO 23 | Motor direction B |
| ESP32-CAM | GPIO (UART) | Color detection data |
| IMU SDA | GPIO 6 | I2C data |
| IMU SCL | GPIO 7 | I2C clock |

---

## 💻 Source Code

The main program runs on the **XIAO ESP32-C6**, which receives RPLiDAR distance data, processes it for navigation decisions, and controls both the steering servo and drive motor.

### What the system does

- 📡 Reads 360° distance data from the RPLiDAR A1M8
- 🏎️ Controls drive motor speed via TB6612FNG motor driver
- 🎯 Controls MG90 steering servo for directional adjustments
- ↩️ Detects corners and executes turns automatically
- 📏 Maintains target distance from track walls
- 🔄 Applies PID correction to stay centered between walls
- 🔢 Counts completed laps and stops after 3
- 🎨 Receives red/green pillar color data from ESP32-CAM (Obstacle Challenge)

### Open Challenge — How it works

The robot uses the RPLiDAR as its only sensor. On each scan cycle it measures distances to the left wall, right wall, and front wall. A proportional controller calculates the steering correction needed to stay centered. When a front wall is detected below the threshold distance, the robot executes a corner turn in the pre-determined direction and increments the corner counter. After 12 corners (3 laps × 4 corners), the robot stops autonomously in the finish section.

### Obstacle Challenge — How it works

Same wall-following base as the Open Challenge, with the addition of the ESP32-CAM. Before passing each traffic sign pillar, the camera classifies its color. A red pillar must be passed on the right; a green pillar must be passed on the left. After completing 3 laps, the robot locates the magenta parking lot boundaries using the LiDAR and executes a parallel parking maneuver.

---

## 🏁 Challenge Summary

| Feature | Open Challenge | Obstacle Challenge |
|---------|---------------|-------------------|
| Laps | 3 | 3 |
| Traffic signs | ✗ | ✓ Red & Green pillars |
| Parking | ✗ | ✓ Parallel parking |
| Primary sensor | RPLiDAR | RPLiDAR + ESP32-CAM |
| Max points | 30 | 62 |

---

## 🔧 Build & Setup

### 1. 🖨️ Print the chassis
Download the 3D models from the `models/` folder and print the required parts.

### 2. ⚙️ Assemble the robot
Mount the motors, servo, LiDAR, and electronics onto the chassis following the design layout.

### 3. 🔌 Wire the electronics
Follow the wiring diagram at `schemes/Esquema_electronica.jpg` to connect all components.

### 4. 💻 Upload the code
Connect each microcontroller to a computer and flash the corresponding programs:
- `src/main/main.cpp` → flash to XIAO ESP32-C6 using Arduino IDE
- `src/main/navigation.py` → run on host computer or single-board computer

### 5. 🔋 Power on and test
Insert the LiPo battery, flip the power switch, and verify all systems initialize correctly before placing the robot on the track.

---

## 📷 Vehicle Photos

| Front | Front-2 | Right |
|:---:|:---:|:---:|
| ![Front](v-photos/Front.jpeg) | ![Front_2](v-photos/Front_2.jpeg) | ![Left](v-photos/Left.jpeg) |
| **Right-2** | **Top** | **Isometric** |
| ![Right](v-photos/Right.jpeg) | ![Top](v-photos/Top.jpeg) | ![Isometric](v-photos/Isometric.jpg) |

---

## 👕 Team Photos

| Team | Paco | Emi | Oliver |
|:---:|:---:|:---:|:---:|
|<div align="center"><img src="v-photos/Team.jpg" width="250" height="250"></div>) | <div align="center"><img src="t-photos/FrancisoCastillo.jpeg" width="250" height="250"></div>|<div align="center"><img src="t-photos/EmilianoCanche.jpeg" width="250" height="250"></div> | <div align="center"><img src="t photos/OliverMascareño.jpeg" width="250" height="250"></div> |



---

## 🎬 Videos

| Challenge | Link |
|-----------|------|
| Open Challenge — No obstacles | [Watch](video/Video1_SinObs.mp4) |
| Obstacle Challenge | *Coming soon* |
