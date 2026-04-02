SIMULACIÓN

PROYECTO: Entorno de Simulación con Dirección Ackermann
SISTEMA: ROS 2 Jazzy / Gazebo Sim (Harmonic)
EQUIPO: MSI Thin 15 B13VF / Chasis Aston Martin Green

------------------------------------------------------------
1. COMPILACIÓN DEL WORKSPACE (BUILD)
------------------------------------------------------------
Es vital realizar una compilación limpia si hubo cambios en el 
URDF o en las texturas de la pista:

$ cd ~/C-ROS/AutoModelCar/simulacion_ws
$ colcon build --symlink-install
$ source install/setup.bash

------------------------------------------------------------
2. EJECUCIÓN Y SELECCIÓN DE MODOS (EL "SWITCH")
------------------------------------------------------------
El launch principal permite alternar entre dos configuraciones 
de red de datos usando el argumento 'mode'. 

¡IMPORTANTE!: Siempre presiona "PLAY" en la GUI de Gazebo tras 
lanzar para activar el flujo de paquetes.

------------------------------------------------------------
MODO 1: PERCEPCIÓN COMPLETA (CONTROL FÍSICO + SENSORES)
------------------------------------------------------------
Uso: Pruebas de navegación autónoma y visión artificial.
Comando:
$ ros2 launch carro_simulacion car_sim.launch.py mode:=1

Configuración:
- Sensores Virtuales: Lidar 360 y Cámara RealSense ACTIVOS.
- Tráfico: Datos fluyendo en /model/carro/scan y /camera/image.
- Control: El chasis obedece al tópico /model/carro/cmd_vel 
  (enviado desde un control físico o teclado).

------------------------------------------------------------
MODO 2: TELEMETRÍA POR ENCODERS (BAJO NIVEL)
------------------------------------------------------------
Uso: Calibración de tracción y pruebas de odometría pura.
Comando:
$ ros2 launch carro_simulacion car_sim.launch.py mode:=2

Configuración:
- Sensores Virtuales: Nodos de Lidar y Cámara DESACTIVADOS 
  (Ahorro de recursos en la GPU).
- Tráfico: Los nodos de percepción no publican datos.
- Control: El movimiento es gobernado por los mensajes de 
  los ENCODERS de los motores (motor_msgs).

------------------------------------------------------------
3. MONITOREO EN RVIZ2 (SÓLO MODO 1)
------------------------------------------------------------
Para visualizar la "Capa de Presentación" de los sensores:

1. Ejecutar: $ rviz2
2. Fixed Frame: Cambiar a "base_link"
3. Add -> LaserScan (Topic: /model/carro/scan, Policy: Best Effort)
4. Add -> Image (Topic: /camera/image, FOV: 73.8°)

------------------------------------------------------------
4. ORDEN DE "STACKING" (ENCADENAMIENTO)
------------------------------------------------------------
Para evitar errores de rutas, el "ruteo" en cada terminal es:
1. $ source /opt/ros/jazzy/setup.bash
2. $ source ~/C-ROS/AutoModelCar/motores/install/setup.bash
3. $ source ~/C-ROS/AutoModelCar/simulacion_ws/install/setup.bash

------------------------------------------------------------
5. ESPECIFICACIONES TÉCNICAS
------------------------------------------------------------
- Entorno: Pista de 12m x 8m (Textura pista.png)
- Cámara: Calibración RealSense (FOV 1.288 rad)
- Lidar: 640 muestras / 360 grados (Blue Link)
- Chasis: Masa 2.5kg / Fricción Mu 0.9

------------------------------------------------------------
6. TROUBLESHOOTING (MANTENIMIENTO)
------------------------------------------------------------
- ¿No reconoce el modo?: Verifica que el car_sim.launch.py 
  tenga declarada la lógica de IfCondition.
- ¿Warning de TFs?: Asegúrate de que el bridge base para 
  /tf esté activo en ambos modos.
- ¿Lidar vacío?: Confirma <always_on> en 1 dentro del URDF.

============================================================
Jonathan Jason Medina Martinez - OPEN SOURCE - UPIITA
============================================================
