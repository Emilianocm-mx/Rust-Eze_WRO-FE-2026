# 🤖 Rust-eze — WRO 2026 Future Engineers

<div align="center">

![WRO](https://img.shields.io/badge/WRO-2026-FF6600?style=for-the-badge)
![Category](https://img.shields.io/badge/Categoría-Future%20Engineers-blue?style=for-the-badge)
![Country](https://img.shields.io/badge/País-México-green?style=for-the-badge)
![Lang](https://img.shields.io/badge/Lang-C%2B%2B%20%7C%20Python-yellow?style=for-the-badge)

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

<p align="center">
  <img src="t-photos/Equipo.jpeg" width="400"/>
</p>

---

## 🏗️ Arquitectura del sistema

El robot usa una arquitectura de **dos controladores** que se comunican por serial:

```
┌─────────────────────┐        Serial (USB)        ┌──────────────────────┐
│   Raspberry Pi 4    │ ─────────────────────────► │      ESP32           │
│                     │                             │                      │
│  · RPLidar A1/A2    │   Envía ángulo de giro      │  · Driver TB6612FNG  │
│  · Lógica de        │   (0–180°)                  │  · Motor DC tracción │
│    navegación       │                             │  · Servo dirección   │
│  · Python           │                             │  · C++ (Arduino)     │
└─────────────────────┘                             └──────────────────────┘
```

**Flujo de decisión:**
1. El RPLidar escanea 360° y mide la distancia a la pared izquierda (~90°) y derecha (~270°)
2. La Raspberry calcula el ángulo de corrección para mantener el robot centrado en la pista (ancho objetivo: 1000 mm)
3. Envía el ángulo por serial al ESP32
4. El ESP32 controla el motor y el servo en consecuencia

---

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

⚠️ **Pendiente** — agregar fotos en `v-photos/` con los siguientes ángulos requeridos por WRO:
`front.jpg` · `back.jpg` · `left.jpg` · `right.jpg` · `top.jpg` · `bottom.jpg`

---

## 📐 Esquemas

⚠️ **Pendiente** — agregar en `schemes/`:
- Diagrama de conexiones ESP32 ↔ TB6612FNG ↔ Motor
- Diagrama de conexiones Raspberry Pi ↔ RPLidar
- Vista general del cableado

---

## 🎬 Video

| Archivo | Descripción |
|---|---|
| `Video1_SinObs.mp4` | Ronda de prueba sin obstáculos |
| _por agregar_ | Ronda con obstáculos |

---

## 📓 Engineering Journal

Documentación del proceso de diseño e iteraciones en [`docs/engineering-journal.md`](docs/engineering-journal.md).

---

## 📜 Licencia

Uso educativo y competitivo — [MIT License](LICENSE).
