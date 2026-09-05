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
├── schemes/                
├── docs/
│   └── engineering-journal.md
├── t-photos/               # Fotos del equipo
│   ├── Equipo.jpeg
│   ├── Paco.jpeg
│   ├── Emi.jpeg
│   └── Oliver.jpeg
├── v-photos/               
├── video/
│   └── Video1_SinObs.mp4   # Demo — ronda sin obstáculos
├── models/                 
└── requirements.txt
```

---

## ⚡ Alimentación

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

El programa principal controla el funcionamiento del robot. El **ESP32-C6** recibe y procesa las lecturas del **LiDAR** para conocer las distancias del entorno y utiliza esta información para controlar el movimiento del robot y la dirección mediante el servo.

El sistema permite:

* 📡 Obtener las lecturas de distancia del LiDAR.
* 🚗 Controlar el motor de avance mediante el Mini H-Bridge.
* 🎯 Controlar el servo de dirección **MG90**.
* ↩️ Detectar curvas y determinar cuándo realizar un giro.
* 📏 Mantener una distancia determinada respecto a las paredes.
* 🔄 Corregir automáticamente la dirección durante el recorrido mediante un sistema PID.
* 🛣️ Detectar cuándo el robot termina un giro y vuelve a la trayectoria.

### Conexiones principales

| Componente | ESP32-C6 | Función              |
| ---------- | -------: | -------------------- |
| LiDAR RX   |  GPIO 19 | Recepción de datos   |
| LiDAR TX   |  GPIO 16 | Transmisión de datos |
| Motor      |  GPIO 20 | Control del motor    |
| Servo MG90 |   GPIO 0 | Control de dirección |
| PWMA       |  GPIO 22 | Velocidad del motor  |
| AIN1       |   GPIO 2 | Dirección del motor  |
| AIN2       |  GPIO 23 | Dirección del motor  |

### Funcionamiento del programa

Al iniciar, el **ESP32-C6** configura el motor, el servo y el LiDAR. Después de iniciar el LiDAR, el robot realiza una lectura inicial del entorno y comienza a avanzar.

Durante el recorrido, el LiDAR obtiene diferentes mediciones dependiendo del ángulo. Estas mediciones permiten identificar:

* Distancia frontal.
* Distancia de los lados.
* Distancia durante los giros.
* Espacios abiertos.
* Posición del robot respecto a las paredes.

Con estos datos, el programa determina cuándo debe realizar un giro y mueve el **servo MG90** hacia la izquierda o derecha. Después del giro, el sistema vuelve a analizar las distancias para continuar con la navegación.

El control de dirección utiliza un **PID**, que realiza pequeñas correcciones en el servo para mantener al robot en una posición adecuada durante las partes rectas del recorrido.


---

## 🔄 Funcionamiento general

Al activar el switch, comienza a suministrarse energía al sistema. El ESP32-C6 inicia los diferentes componentes y el LiDAR comienza a obtener información de las distancias alrededor del robot.

Después de un breve periodo de inicialización, se activa el motor de tracción. Mientras el robot avanza, el LiDAR analiza las distancias y la posición de los bloques, permitiendo que el ESP32-C6 determine cómo debe orientarse el robot y ajuste el MG90 para realizar los giros.

Para el segundo reto, la ESP32-S3 Sense Mini con ESP-CAM se encarga de identificar los colores de los bloques. Esta información se combina con la información espacial obtenida mediante el LiDAR para que el robot pueda tomar decisiones durante la navegación.

---

## 🏁 Separación de los dos retos

Reto 1 — Navegación con LiDAR

El robot utiliza el LiDAR como sistema principal de percepción para medir las distancias a su alrededor y determinar su posición dentro de la pista. El ESP32-C6 procesa esta información y controla el sistema de dirección para realizar la vuelta de manera eficiente.

Reto 2 — Detección de bloques

Para el segundo reto se incorpora la ESP32-S3 Sense Mini con ESP-CAM, encargada de detectar los colores de los bloques. El LiDAR proporciona información sobre la posición y distancia de los bloques, mientras que la cámara permite identificar su color. La combinación de ambas fuentes de información permite al robot tomar decisiones de navegación.

---

## 🛠️ Componentes principales

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

## 🔧 Instalación y armado

Para construir y poner en funcionamiento el robot, sigue los siguientes pasos:

### 1. 🖨️ Imprimir el chasis

Descarga los modelos 3D disponibles en la carpeta [`models/`](models/) e imprime las piezas necesarias para construir el chasis del robot.

### 2. ⚙️ Ensamblar el robot

Coloca los motores, el servo, el LiDAR y los componentes electrónicos en el chasis siguiendo el diseño del robot.

### 3. 🔌 Conectar la electrónica

Realiza las conexiones entre el **ESP32-C6**, **LiDAR**, **ESP32-S3 Sense Mini**, **MG90**, **Mini H-Bridge**, motor y **LM2596** siguiendo el diagrama disponible en [`schemes/Electronic_scheme`](schemes/Electronic_scheme.jpg).

### 4. 💻 Cargar el programa

Conecta los microcontroladores a una computadora y carga los programas correspondientes.


### 5. 🔋 Encender y probar

Coloca la batería LiPo, activa el switch y verifica que todos los sistemas funcionen correctamente.

El robot estará listo para comenzar las pruebas de navegación y detección de bloques.




---

## 📷 Fotos del vehículo
---
Front: [`models/`](v-photos/Front.jpeg/)
Front_2: [`models/`](v-photos/Front_2.jpeg/)
Left: [`models/`](v-photos/Left.jpeg/)
Right: [`models/`](v-photos/Right.jpeg/)
Top: [`models/`](v-photos/Top.jpeg/)

---
