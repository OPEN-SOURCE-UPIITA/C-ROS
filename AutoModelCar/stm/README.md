# Proyectos STM32 - C-ROS (AutoModelCar)

¡Bienvenido al código base de bajo nivel para el RosRobotController V1.1 (STM32F407VET6)! 

Aquí encontrarás los proyectos de prueba esenciales para validar el hardware del robot (Motores DC, Servomotores, LEDs, etc.).

**⚠️ IMPORTANTE: ¿Dónde están los archivos .hex?**
Por buenas prácticas de desarrollo colaborativo y para mantener el repositorio ligero, **NO subimos archivos binarios compilados** (es decir, no verás las carpetas `Debug/`). Cada desarrollador debe compilar el código fuente en su propia máquina para generar el archivo ejecutable (`.hex`).

---

## 🛠️ Requisitos
* **STM32CubeIDE:** Entorno oficial para editar y compilar el código C.
* **stm32flash:** Herramienta de terminal para Linux usada para cargar el firmware por puerto serial.

---

## 🚀 Instrucciones de Uso

### Paso 1: Compilar el código (Generar el .hex)
1. Abre **STM32CubeIDE**.
2. Ve a `File` -> `Open Projects from File System...`, haz clic en *Directory* y selecciona la carpeta del proyecto que deseas probar (ej. `MotorDC`).
3. Haz clic en el ícono del **Martillo 🔨** (Build Debug) en la barra de herramientas superior.
4. Verifica en la consola inferior que el proceso termine con el mensaje `Build Finished. 0 errors`.
   * *Magia:* Esto generará automáticamente una carpeta llamada `Debug/` en tu directorio, la cual contiene el codiciado archivo `.hex`.

### Paso 2: Cargar el código al Microcontrolador (Flasheo)
1. Conecta el RosRobotController a tu computadora usando el cable USB-C (puerto central).
2. Pon el microcontrolador en **Modo Bootloader** mediante hardware:
   * Mantén presionado el botón **BOOT0** (junto a los pines GPIO).
   * Presiona y suelta el botón **RESET**.
3. Abre una terminal de Linux **dentro de la carpeta `Debug/`** que acabas de generar.
4. Ejecuta el comando de flasheo (reemplaza el nombre del archivo según el proyecto):

   ```bash
   stm32flash -w _NombreDelProyecto_.hex -v -g 0x0 -R /dev/ttyACM0
   ```
5. Suelta el botón **BOOT0** y presiona **RESET** si es necesario.
(Nota: Asegúrate de que el puerto asignado a la placa sea /dev/ttyACM0. Si falla con error "Read Protected", primero ejecuta el comando de borrado: sudo stm32flash -k /dev/ttyACM0).

## Nuevo proyecto
Para la creación de nuevos programas consultar el pdf guía: _Guía programación STM32_ dónde se detalla cómo funciona la creación de un proyecto usando las herramientas oficiales de STM.
