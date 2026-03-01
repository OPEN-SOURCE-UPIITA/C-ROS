# 🤖 Control de Motores y Odometría con ESP32-S3 y micro-ROS

Este repositorio contiene la implementación de un nodo de ROS 2 ejecutado en un microcontrolador ESP32-S3. Permite el control de velocidad y dirección (cinemática diferencial) de motores DC y la lectura precisa de encoders de cuadratura utilizando **micro-ROS**.

El proyecto está diseñado para ser la capa de hardware de bajo nivel (Firmware) para robots móviles, recibiendo comandos de velocidad (`cmd_vel`) y publicando el estado de los motores (ticks del encoder).

## 🛠️ Hardware Soportado y Requisitos

* **Microcontrolador:** ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM Octal). *Nota: Es crucial usar esta variante o ajustar la configuración de memoria en el IDE.*
* **Motores:** Motor DC JGA25-370 (12V, 350RPM) con Encoder de efecto Hall (Resolución: 341.2 PPR).
* **Driver de Potencia:** Puente H estándar (L298N, TB6612FNG, etc.).

## ⚙️ Configuración del Entorno (Arduino IDE)

Para que micro-ROS y las interrupciones por hardware del encoder funcionen correctamente en el ESP32-S3, es **estrictamente necesario** configurar el Arduino IDE con los siguientes parámetros en el menú `Herramientas` (Tools):

* **Board:** ESP32S3 Dev Module
* **USB CDC On Boot:** `Disabled` *(Mantiene estable el puerto COM por UART físico).*
* **Flash Size:** `16MB (128Mb)`
* **Partition Scheme:** `16M Flash (3MB APP/9.9MB FATFS)` *(micro-ROS requiere particiones grandes).*
* **PSRAM:** `OPI PSRAM`
* **Upload Mode:** `UART0 / Hardware CDC`

### 📚 Dependencias (Librerías)
Debes instalar las siguientes librerías desde el Gestor de Librerías de Arduino:
1.  **`micro_ros_arduino`**: Para la comunicación con ROS 2.
2.  **`ESP32Encoder`** (por Kevin Harrington): Para la lectura del encoder por hardware (módulo PCNT), evitando la pérdida de pulsos que ocurre con interrupciones de software tradicionales.

## 🔌 Pinout (Conexiones)

| Componente | Pin ESP32-S3 | Descripción |
| :--- | :--- | :--- |
| Puente H (ENA) | `GPIO 4` | Señal PWM para velocidad |
| Puente H (IN1) | `GPIO 5` | Dirección Motor 1 |
| Puente H (IN2) | `GPIO 6` | Dirección Motor 2 |
| Encoder (Fase A) | `GPIO 10` | Canal A del sensor Hall |
| Encoder (Fase B) | `GPIO 11` | Canal B del sensor Hall |

## 🚀 Despliegue y Uso

### 1. Consideraciones del Código (Troubleshooting)
* **Zona Muerta (Deadband):** El código incluye un mapeo de PWM (170 a 255) para compensar la zona muerta del motor JGA25-370, asegurando que responda a comandos de baja velocidad.
* **Error de compilación `puType`:** Al configurar las resistencias internas para el encoder, asegúrate de usar la sintaxis actualizada de la librería. El código utiliza `ESP32Encoder::useInternalWeakPullResistors = puType::up;`. Si utilizas una versión antigua, podría ser necesario cambiarlo a `UP` o `esp32_port_logic_t::UP`.

### 2. Ejecución del Agente micro-ROS
Para que el ESP32 se comunique con la red de ROS 2 en tu computadora principal (Raspberry Pi o PC), debes ejecutar el agente. Conecta el ESP32 por USB (UART) e inicia el contenedor de Docker:

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
(Asegúrate de cambiar /dev/ttyUSB0 por el puerto correcto de tu microcontrolador)
```

## 3. Tópicos de ROS 2
Una vez conectado, el nodo interactúa con los siguientes tópicos:

* **Suscripción** (`/cmd_vel`): Recibe mensajes tipo `geometry_msgs/msg/Twist`.
  * Aplica la cinemática inversa para calcular la velocidad de cada rueda según la fórmula:

$$V_{left,right} = V_x \pm \frac{\omega_z \cdot L}{2}$$

  (Donde $L$ es la distancia entre ruedas).

* **Publicación** (`/motor_ticks`): Publica mensajes tipo `std_msgs/msg/Int32` con el conteo absoluto del encoder a una frecuencia de 20Hz.

**Desarrollado para la comunidad Open Source.**

