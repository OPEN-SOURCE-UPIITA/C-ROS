
#  Sistema de Control de Motores con ROS 2 + STM32

Este proyecto implementa una arquitectura distribuida basada en **ROS 2** para controlar motores DC y un servo mediante un **STM32** conectado por puerto serial.

El sistema separa la lógica de alto nivel (ROS 2) del control de hardware (STM32), permitiendo fácil extensión, mantenimiento y pruebas.

---

##🏗️ Arquitectura General

[ Nodos de Control ] --> (/motor_command) --> [ Nodo Controlador Serial ] --> (UART) --> [ STM32 ] --> [ Motores ]



---

# 🧩 Componentes Principales

## 1️⃣ Mensaje Personalizado: `MotorCommand`

Define la estructura de los comandos enviados al robot.

Contiene tres campos:

- `direction`  
  - `0 = Stop`
  - `1 = Adelante`
  - `2 = Atrás`

- `speed`  
  - Rango: `0 – 100`

- `servo`  
  - `0 = Centro`
  - `1 = Izquierda`
  - `2 = Derecha`

Cualquier nodo puede publicar comandos usando este mensaje.

---

## 2️⃣ Nodo Controlador Serial (`stm32_controller`)

- Se suscribe al tópico `/motor_command`
- Empaqueta los datos en una trama binaria de 7 bytes
- Envía la trama por UART a **115200 baudios**
- Publica a **20 Hz**
- Implementa un watchdog de seguridad (200 ms)

### 📦 Formato de Trama

[0xAA][0x55][0x01][direction][speed][servo][0xFF]

- `0xAA 0x55` → Cabecera
- `0x01` → ID de comando
- `0xFF` → Byte de terminación

---

## 3️⃣ Nodo de Teleoperación por Teclado

Publica comandos en `/motor_command` en tiempo real.

Controles:

- ⬆️ Flecha arriba → Adelante
- ⬇️ Flecha abajo → Atrás
- ⬅️ Flecha izquierda → Servo izquierda
- ➡️ Flecha derecha → Servo derecha
- `Espacio` o `S` → Stop
- `C` → Centrar servo

Publica continuamente el último estado para mantener control estable.

---

## 4️⃣ Nodo de Rutina Autónoma

Ejecuta secuencias programadas publicando en `/motor_command`.

Ejemplo:
- Describir un “8”
- Alternar dirección del servo
- Cambiar velocidad dinámicamente

---

## 5️⃣ Firmware STM32

Responsable del control físico de los motores.

Funciones:

- Recepción UART por interrupción
- Validación de trama
- Conversión de velocidad (0–100 → PWM 0–1000)
- Generación de señales PWM
- Control de LEDs de estado
- Watchdog interno

### 🔄 Lógica de Control

| Dirección | Acción PWM |
|-----------|------------|
| Adelante  | Canal A activo, B = 0 |
| Atrás     | Canal invertido |
| Stop      | Ambos canales al 100% (freno activo) |

---

# 🔄 Flujo de Datos

## 1️⃣ Generación de Comando
El comando puede provenir de:
- Teclado
- Rutina autónoma
- Cualquier nodo ROS

Se publica en `/motor_command`.

## 2️⃣ Procesamiento en Nodo Serial
- Extrae dirección, velocidad y servo
- Empaqueta en trama binaria
- Envía por UART

## 3️⃣ Recepción en STM32
- Verifica cabecera y terminador
- Extrae valores
- Configura PWM
- Ejecuta acción física

---

# 🛡️ Seguridad

- Watchdog en nodo serial (200 ms)
- Watchdog en STM32
- Validación de trama
- Freno activo en estado Stop

Si se pierde comunicación, los motores se detienen automáticamente.

---

# ⚙️ Parámetros Técnicos

- Frecuencia de control: **20 Hz**
- UART: **115200 baudios**
- Trama: **7 bytes**
- Watchdog: **200 ms**
- PWM: **0–1000**
- Velocidad lógica: **0–100**

---

# 📡 Tópicos ROS

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/motor_command` | `MotorCommand` | Comando de motores y servo |

---

# 🚀 Casos de Uso

- Teleoperación manual
- Rutinas autónomas
- Integración con navegación
- Control por joystick
- Expansión con sensores

---

# 🔧 Extensiones Futuras

- Integración con visión (OpenCV / YOLO)
- Publicación de estado del STM32
- Control PID cerrado
- Integración con SLAM
- Migración a micro-ROS

---

# 📌 Filosofía del Diseño

Separar:

- 🧠 Inteligencia → ROS 2  
- ⚙️ Control físico → STM32  

Permite escalar el sistema sin modificar el firmware cada vez que cambia la lógica de control.
