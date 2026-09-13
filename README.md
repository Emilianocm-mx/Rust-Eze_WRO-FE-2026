## 🤖 Rust-eze — WRO 2026 Future Engineers

![WRO](https://img.shields.io/badge/WRO-2026-blue)
![Category](https://img.shields.io/badge/Category-Future%20Engineers-green)
![Country](https://img.shields.io/badge/Country-Mexico-red)
![Lang](https://img.shields.io/badge/Code-C%2B%2B%20%7C%20Python-yellow)

Official repository of team **Rust-eze** for the WRO 2026 Future Engineers category. Here you will find the source code of our robot, technical documentation, hardware schematics, and multimedia material from our autonomous robot.

---

## 📑 Table of contents

* [👥 Team](#team)
* [📷 Vehicle Photos](#vehicle-photos)
* [⚙️ Mechanical Design Process](#mechanical)
* [⚡ Electrical System](#electrical)
* [🔌 Wiring Diagram](#wiring)
* [🛠️ Components](#components)
* [💻 Source Code](#source-code)
* [🔧 Build & Setup](#build)
* [👕 Team Photos](#team-photos)

---

<a id="team"></a>
## 👥 Team

| Name | Role |
|------|------|
| Emi | Mechanical |
| Oliver | Electronics |
| Paco | Programmer |
| Leo | Coach |

---

<a id="vehicle-photos"></a>
## 📷 Vehicle Photos

| Front | Front-2 | Right |
|:---:|:---:|:---:|
| ![Front](v-photos/Front.jpeg) | ![Front_2](v-photos/Front_2.jpeg) | ![Left](v-photos/Left.jpeg) |
| **Right-2** | **Top** | **Isometric** |
| ![Right](v-photos/Right.jpeg) | ![Top](v-photos/Top.jpeg) | ![Isometric](v-photos/Isometric.jpg) |

---

<a id="mechanical"></a>
## ⚙️ Mechanical Design Process

Our mechanical design focuses on stability, and precise maneuverability to tackle the high-speed requirements of the WRO Future Engineers challenges. The entire robot was designed from scratch using CAD software and manufactured via 3D printing.

### 1. Chassis & Weight Distribution
* **Material:** Printed in PLA/PETG for a great balance between rigidity and lightweight performance.
* **Low Center of Gravity:** Heavier components, such as the LiPo battery and the Pololu motor, are mounted as low as possible to prevent the robot from tipping over during sharp corners.
* **Modular Design:** The chassis is divided into easily replaceable sections (front steering assembly, main body, and rear drivetrain) to allow quick repairs during competition days. Also, our CAD desing is basically 2 pieces that consist of a top_cover and main_chassis with each mechanisism working independently.

| Top | Chassis |
|:---:|:---:|
| <div align="center"><img src="v-photos/top.png" width="550" height="550"></div> | <div align="center"><img src="v-photos/chasis.png" width="550" height="550"></div> |



### 2. Steering System (Ackermann Geometry)
To ensure smooth cornering and minimize tire scrubbing, we implemented an **Ackermann steering mechanism** which is the same type that comercial cars use.
* Actuated by the **MG90 Micro Servo** and a steering system assembled with 3 M2 screws ensure that our ackermann the type steering can work efficiently at all times.
* This geometry is critical for maintaining traction and speed while navigating the tight turns of the challenge in general.

| Our Steering | Ackermann Steering Formula |
|:---:|:---:|
| <div align="center"><img src="v-photos/stearing.png" width="550" height="550"></div> | <div align="center"><img src="v-photos/ackermann_steering.png" width="425" height="425"></div> |


### 3. Drivetrain & Transmission
* **Rear-Wheel Drive (RWD):** Driven by a single **N20 Pololu 300 RPM DC Motor**. While it was not the fist motor we used, the different design itterations helped us find the perfect motor to power out robot. 
* **Power Transfer:** We designed a custom structure with bearings in order to use the lego differential (pieces 65413 and 65414) connecting the motor to the solid rear axle. This gear ratio was calculated to provide the perfect balance between top speed on the straightaways and high torque for the Obstacle Challenge maneuvers (like parallel parking). We opted for a lego build design due to the risks that a 3D printed one would give us.

### 4. Sensor Integration
* **RPLiDAR Tower:** Elevated and centrally mounted to guarantee a 360° unobstructed field of view, preventing any chassis parts from creating blind spots. It is mounted with M2.5 screws with perfectly designed holes in the cassis.
* **Camera Mount:** The ESP32-CAM is mounted in front of the robot with a simple slot on the cassis so that it can still be taken out easily without compromising stability.

### 🔄 Design Iterations

| Version | Focus Area | Key Improvements |
|---------|------------|------------------|
| **V1.0** | Functionality and weight reduce | Since we used to depend on a RPI and a PWR BANK, the chassis needed to be light and sturdy in order hold up with the weight of this components. |
| **V2.0** | Steering Optimization | Implemented Ackermann geometry, with a MG90 servo in order to provide more torque for our robot. |
| **V3.0** | Final Frame | We changed to lighter components (XIAO C6 and LiPo battery) so our final chassis ended up being more sturdy and easy to assemble then at the beggining. |

---

<a id="electrical"></a>
## ⚡ Electrical System
![Electronic Scheme](schemes/Electronic_Scheme.jpg)

### Power Architecture

```
LiPo Battery 7.4V
        │
    [Switch]
        │
    ┌───┴──────────────────┐
    │                      │
    ▼                      ▼
TB6612FNG                LM2596
Motor Driver           Step-down
  (7.4V)                5V out
    │                      │
    ▼                ┌──────┼──────────┐
Pololu Motor         ▼      ▼          ▼
                ESP32-C6  RPLiDAR  ESP32-CAM
```
---

<a id="wiring"></a>
## 🔌 Wiring Diagram
<img width="1024" height="723" alt="image" src="https://github.com/user-attachments/assets/959afa32-fa0c-43f0-8441-2c4025aa1184" />

---

<a id="components"></a>
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
| <div align="center"><img src="v-photos/LiPo.jpg" width="250" height="250"></div> | **LiPo Battery 7.4V 800mAh** | • Main power source for the entire system <br> • Directly feeds motor driver at full voltage <br> • Feeds LM2596 for 5V logic rail <br> • Lightweight for weight-sensitive robot design |
| <div align="center"><img src="v-photos/Switch.jpg" width="250" height="250"></div> | **Power Switch** | • Master on/off switch for the entire system <br> • Required by WRO rules (one switch to power on) <br> • Cuts all power from battery before start |
| <div align="center"><img src="v-photos/IMU.jpg" width="250" height="250"></div> | **BMI160** | • Inertial measurement unit <br> • Provides heading and orientation data <br> • Supports navigation during turns <br> • Connected via I2C (SDA/SCL) to ESP32-C6 |

---

<a id="source-code"></a>
## 💻 Source Code

The main program runs on the **XIAO ESP32-C6**, which processes RPLiDAR distance data in real time, fuses it with IMU heading data, and controls both the steering servo and drive motor through a hybrid PID + yaw control system.

---

### System Capabilities

- 📡 Reads 360° distance data from the RPLiDAR A1M8 via UART, sampling specific angles as named rays (0°, 60°, 90°, 120°, 170°, 180°, 190°, 240°, 270°, 300°, 340°, 20°)
- 🧭 Integrates gyroscope data from the BMI160 at 100 Hz to track heading (yaw) with automatic bias calibration at startup
- 🔄 Runs a **hybrid steering controller**: IMU-based yaw control as primary, with LiDAR wall-following as fallback when IMU is invalid
- 📏 Maintains a dynamic wall distance setpoint that is captured fresh after each corner, adapting to each corridor without forcing a fixed target
- ↩️ Detects corners using a multi-condition state machine: front wall blocked + back clear + lateral opening confirmed + opposite wall closed
- 🔢 Counts completed corners and laps, stopping the motor autonomously after 3 laps (12 corners total)
- 🔁 Applies **IMU drift correction** continuously during straight sections by comparing wall angle against integrated yaw
- 🎨 Receives red/green pillar color data from ESP32-CAM via UART with CRC-validated framing (Obstacle Challenge)
- 🔘 Implements a **two-device start protocol**: the robot waits for a button press signal from the ESP32-CAM before enabling motion, satisfying WRO's one-button start rule without a physical button on the main controller

---

### 🏁 Open Challenge

The Open Challenge uses the RPLiDAR as the primary perception source and the BMI160 as the heading reference. No camera input is used.

**Startup sequence:**
The robot initializes the BMI160, collects 500 gyroscope samples to compute a Z-axis bias offset, then starts the LiDAR and waits for the start signal from the ESP32-CAM button. Once received, it waits 500 ms before enabling motion, giving the LiDAR rotor time to reach full speed and the IMU reference to settle.

**First corridor:**
Before the first corner is completed, `trackDir` is `UNKNOWN`. The robot reads two diagonal rays on each side (60°/120° for right, 300°/240° for left) and selects whichever wall is closer as its reference. It then follows that wall at the measured distance, using parallelism between the front and back rays to stay aligned. A soft-escape maneuver activates if the robot starts too close to a wall, gently steering away until a safe clearance is reached.

**Straight-line control:**
Once a reference wall is selected, the steering loop runs every 20 ms. It computes a desired yaw correction from the wall distance error, rate-limits the setpoint change to avoid sharp inputs, then feeds it into a yaw P+D controller using the IMU. The derivative term damps oscillation using the live gyroscope rate rather than a numerical derivative. While the IMU is valid, this loop runs as primary. If the IMU fails or loses calibration, the system falls back to a pure LiDAR PID on the wall rays directly.

**Drift correction:**
On every straight section, the system compares the geometric angle of the tracked wall (computed from the two diagonal rays) against the IMU's relative yaw. If they differ consistently, a slow integrating correction is applied to `yawOffset`, keeping the IMU reference aligned with reality over time without introducing sudden jumps.

**Corner detection:**
A corner is triggered when all of the following are true simultaneously: a front wall is detected below 1200 mm, the back is clear beyond 1500 mm, the lateral opening on the turn side has been continuously open for at least 100 ms after previously seeing a wall, and the opposite wall confirms a corridor is closed. This multi-condition approach prevents false triggers from partial LiDAR returns or transient reflections.

**Turn execution:**
When a corner is confirmed, the servo goes to full lock and the IMU target is updated by ±90°. The turn completes when the IMU error falls within 4° and the angular rate drops below 35°/s for 60 ms. A 1400 ms timeout serves as a safety fallback. After the turn, the wall distance setpoint is re-captured from the new corridor's diagonal rays, and all PID state (integral, error smoother) is reset to avoid impulse artifacts.

**Lap completion:**
After 12 corners (3 laps × 4 corners), the robot enters `FINAL_APPROACH` state, drives forward for a calibrated time window (currently 800 ms), then cuts the motor and centers the servo, stopping autonomously in the finish section.

---

### 🚧 Obstacle Challenge

*Implementation in progress. The ESP32-CAM color detection pipeline and parking maneuver logic are currently under development.*

The Obstacle Challenge will extend the Open Challenge base with:
- Color classification of red and green pillars using the ESP32-CAM
- Steering offset logic to pass red pillars on the right and green pillars on the left
- Detection of the magenta parking lot boundaries using the LiDAR after lap 3
- Parallel parking maneuver into the designated area

---

### Pin Reference

| Component | ESP32-C6 Pin | Function |
|-----------|-------------|----------|
| LiDAR RX | GPIO 19 | UART data from LiDAR |
| LiDAR TX | GPIO 16 | UART data to LiDAR |
| LiDAR motor | GPIO 20 | PWM speed control |
| Servo MG90 | GPIO 0 | Steering PWM (16-bit LEDC) |
| Motor PWMA | GPIO 22 | Drive motor speed |
| Motor AIN1 | GPIO 23 | Drive motor direction A |
| Motor AIN2 | GPIO 2 | Drive motor direction B |
| ESP32-CAM RX | GPIO 21 | UART from camera |
| ESP32-CAM TX | GPIO 17 | UART to camera |
| IMU SDA | GPIO 18 | I2C data (BMI160) |
| IMU SCL | GPIO 1 | I2C clock (BMI160) |

---

<a id="challenge-summary"></a>
## 🏁 Challenge Summary

| Feature | Open Challenge | Obstacle Challenge |
|---------|---------------|-------------------|
| Laps | 3 | 3 |
| Traffic signs | ✗ | ✓ Red & Green pillars |
| Parking | ✗ | ✓ Parallel parking |
| Primary sensor | RPLiDAR | RPLiDAR + ESP32-CAM |
| Max points | 30 | 62 |

---

<a id="build"></a>
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

### 5. 🔋 Power on and test
Insert the LiPo battery, flip the power switch, and verify all systems initialize correctly before placing the robot on the track.

---

<a id="team-photos"></a>
## 👕 Team Photos 

| Team | Paco | Emi | Oliver |
|:---:|:---:|:---:|:---:|
| <div align="center"><img src="t-photos/Team.jpg" width="700" height="1000"></div> | <div align="center"><img src="t-photos/FrancisoCastillo.jpeg" width="700" height="1000"></div> | <div align="center"><img src="t-photos/EmilianoCanche.jpeg" width="700" height="1000"></div> | <div align="center"><img src="t-photos/OliverMascareno.jpeg" width="700" height="1000"></div> |





---

<a id="videos"></a>
## 🎬 Videos

| Challenge | Link |
|-----------|------|
| Open Challenge — No obstacles | [Watch](video/Video1_SinObs.mp4) |
| Obstacle Challenge | *Coming soon* |
---
- [Go to top](#-rust-eze--wro-2026-future-engineers)
