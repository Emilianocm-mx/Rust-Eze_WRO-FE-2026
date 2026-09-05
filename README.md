# 🤖 Rust-eze — WRO 2026 Future Engineers

<div align="center">

![WRO](https://img.shields.io/badge/WRO-2026-FF6600?style=for-the-badge)
![Category](https://img.shields.io/badge/Categoría-Future%20Engineers-blue?style=for-the-badge)
![Country](https://img.shields.io/badge/País-México-green?style=for-the-badge)
![Lang](https://img.shields.io/badge/Lang-C%2B%2B%20-yellow?style=for-the-badge)

</div>

> Repositorio oficial del equipo **Rust-eze** para la competencia WRO 2026, categoría Future Engineers. Aquí encontrarás el código fuente, documentación técnica, esquemas de hardware y material multimedia de nuestro robot autónomo.

---

## 👥 Equipo

| Nombre | Rol |
|---|---|
| Emi | _Mecánico_ |
| Oliver | _Eléctrico_ |
| Paco | _Programador_ |
| Leo | _Coach_ |


---

## 🏗️ Arquitectura del sistema

```
                         ┌──────────────────────┐
                         │      ESP32-C6        │
                         │  Control principal   │
                         └──────────┬───────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              │                     │                     │
              ▼                     ▼                     ▼
        ┌───────────┐         ┌───────────┐         ┌───────────┐
        │  LiDAR    │         │ ESP32-S3  │         │  MG90     │
        │           │         │ Sense Mini│         │  Servo    │
        │ Distancia │         │  ESP-CAM  │         │ Dirección │
        └───────────┘         └───────────┘         └───────────┘
              │                     │
              │                     │
              └──────────┬──────────┘
                         │
                         ▼
                  ┌───────────────┐
                  │   Lógica de   │
                  │   navegación  │
                  └───────┬───────┘
                          │
                          ▼
                  ┌───────────────┐
                  │  Mini H-Bridge│
                  │    Driver     │
                  └───────┬───────┘
                          │
                          ▼
                  ┌───────────────┐
                  │ Pololu 300 RPM│
                  │ Motor de      │
                  │ tracción      │
                  └───────────────┘
```



## 📁 Estructura del repositorio

```
wro-rusteze/
├── src/
│   └── main/
│       ├── main.cpp        # Control de motor y servo en ESP32 (C++/Arduino)
│       └── Rasp-PC.py      # Navegación con RPLidar en Raspberry Pi (Python)
├── schemes/                # ⚠️ Pendiente: diagramas eléctricos
├── docs/
│   └── engineering-journal.md
├── t-photos/               # Fotos del equipo
│   ├── Equipo.jpeg
│   ├── Eduardo.jpeg
│   ├── Emi.jpeg
│   └── Oliver.jpeg
├── v-photos/               # ⚠️ Pendiente: fotos del vehículo (6 ángulos)
├── video/
│   └── Video1_SinObs.mp4   # Demo — ronda sin obstáculos
├── models/                 # ⚠️ Pendiente: modelos 3D (STL/STEP)
└── requirements.txt
```

---

## ⚡ Alimentación
---
La batería LiPo alimenta el sistema y el LM2596 se utiliza para reducir el voltaje para los componentes que necesitan 5 V.

```
🔋 Batería LiPo
                 7.4 V
                   │
          ┌────────┴────────┐
          │                 │
          ▼                 ▼
    Mini H-Bridge       LM2596
       7.4 V             ↓ 5 V
          │                 │
          ▼          ┌──────┼─────────────┐
      Motor Pololu   │      │             │
                     ▼      ▼             ▼
                  ESP32-C6 LiDAR      ESP32-S3
                                      Sense Mini
```

---


## 💻 Código fuente

### `src/main/main.cpp` — ESP32 (C++/Arduino)
Controla el **motor de tracción** vía driver TB6612FNG y el **servo de dirección** mediante PWM con la librería `ESP32Servo`.

| Pin | GPIO | Función |
|---|---|---|
| STBY | 23 | Habilitador del driver |
| PWMA | 22 | Velocidad del motor (PWM 1kHz, 8 bits) |
| AIN1 | 21 | Dirección del motor |
| AIN2 | 2 | Dirección del motor |
| servoPin | 4 | Servo de dirección |

El ESP32 espera una señal serial de la Raspberry Pi para iniciar el ciclo de control.

### `src/main/Rasp-PC.py` — Raspberry Pi (Python)
Lee el **RPLidar** y calcula el ángulo de dirección:
- Pared izquierda: lecturas en el rango 80°–100°
- Pared derecha: lecturas en el rango 260°–280°
- Ángulo 90° = recto | < 90° = izquierda | > 90° = derecha
- Ancho de pista objetivo: **1000 mm**

---

## 🔄 Funcionamiento general
---
Al activar el switch, comienza a suministrarse energía al sistema. El ESP32-C6 inicia los diferentes componentes y el LiDAR comienza a obtener información de las distancias alrededor del robot.

Después de un breve periodo de inicialización, se activa el motor de tracción. Mientras el robot avanza, el LiDAR analiza las distancias y la posición de los bloques, permitiendo que el ESP32-C6 determine cómo debe orientarse el robot y ajuste el MG90 para realizar los giros.

Para el segundo reto, la ESP32-S3 Sense Mini con ESP-CAM se encarga de identificar los colores de los bloques. Esta información se combina con la información espacial obtenida mediante el LiDAR para que el robot pueda tomar decisiones durante la navegación.

---

## 🏁 Separación de los dos retos
---
Reto 1 — Navegación con LiDAR

El robot utiliza el LiDAR como sistema principal de percepción para medir las distancias a su alrededor y determinar su posición dentro de la pista. El ESP32-C6 procesa esta información y controla el sistema de dirección para realizar la vuelta de manera eficiente.

Reto 2 — Detección de bloques

Para el segundo reto se incorpora la ESP32-S3 Sense Mini con ESP-CAM, encargada de detectar los colores de los bloques. El LiDAR proporciona información sobre la posición y distancia de los bloques, mientras que la cámara permite identificar su color. La combinación de ambas fuentes de información permite al robot tomar decisiones de navegación.

---

## 🛠️ Componentes principales
---
ESP32-C6 — controlador principal del robot.
ESP32-S3 Sense Mini / ESP-CAM — detección visual de colores.
LiDAR — medición de distancias y detección de obstáculos/bloques.
Motor Pololu 300 RPM — sistema de tracción.
Mini H-Bridge — control del motor.
MG90 — dirección del robot.
LM2596 — regulación de voltaje a 5 V.
Batería LiPo 7.4 V — alimentación.
Switch — activación del sistema.

---

## 🔧 Instalación y uso

### Raspberry Pi

```bash
git clone https://github.com/TU_USUARIO/wro-rusteze.git
cd wro-rusteze
pip install -r requirements.txt
python src/main/Rasp-PC.py
```

### ESP32 (PlatformIO o Arduino IDE)

1. Abre `src/main/main.cpp` en PlatformIO o Arduino IDE
2. Instala la librería `ESP32Servo`
3. Selecciona la placa ESP32 y el puerto correcto
4. Sube el código

---

## 📦 Dependencias Python

```
pyserial
pyrplidar
```

> Ver `requirements.txt` para versiones exactas.

---

## 📷 Fotos del vehículo


---
