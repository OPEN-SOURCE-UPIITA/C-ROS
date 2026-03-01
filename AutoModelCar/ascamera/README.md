# 📷 Driver de Cámara RGB-D (ASCamera)

El paquete **ascamera** permite la integración de una cámara RGB-D en **ROS 2**.  
Proporciona imágenes RGB, mapas de profundidad y nubes de puntos **PointCloud2**
para aplicaciones de visión y percepción 3D.

---

## 📦 Contenido del Paquete

- Publicación de imagen RGB
- Publicación de mapa de profundidad
- Publicación de nube de puntos 3D
- Soporte para arquitecturas **x86_64** y **ARM (Raspberry Pi)**

---

## 🛠️ Instalación de Dependencias (CRÍTICO)

⚠️ Este driver utiliza **librerías propietarias** que deben instalarse manualmente
antes de compilar.  
Si se omite este paso, el paquete **no funcionará**.

---

### 💻 PC / Laptop (x86_64)

1. Desde tu workspace, ve a la carpeta de librerías:

```bash
cd src/ascamera/libs/lib/x86_64-linux-gnu/
```
2. Copia las librerías al sistema y actualiza la caché:

```bash
sudo cp *.so /usr/lib/
sudo ldconfig
```

🍓 Raspberry Pi / ARM (aarch64)


```bash
cd src/ascamera/libs/lib/aarch64-linux-gnu/
sudo cp *.so /usr/lib/
sudo ldconfig
```

## ⚙️ Configuración USB (Reglas udev)

Para usar la cámara sin sudo y evitar problemas de permisos:

```bash
cd src/ascamera/scripts
sudo bash create_udev_rules.sh
```
Si es necesario, desconectar y conectar la cámara para el funcionamiento.

## 🚀 Compilación del Paquete

```bash
cd ~/tu_workspace
colcon build --packages-select ascamera
source install/setup.bash
```

## ▶️ Ejecución

```bash
ros2 launch ascamera hp60c.launch.py
```

## 📡 Tópicos Principales

- /ascamera_hp60c/camera_publisher/rgb0/image (Imagen RGB)
- /ascamera_hp60c/camera_publisher/depth0/image_raw (Mapa de profundidad)
- /ascamera_hp60c/camera_publisher/depth0/points (Nube de puntos 3D)


## 🖥️ Visualización (RViz2)

Para visualizar la Nube de Puntos (PointCloud2) se utiliza la herramienta RViz2.


1. Ejecutar RViz2
```bash
ros2 run rviz2 rviz2
```
2. Configuración Global (Panel Izquierdo)
   - En Fixed Frame colocar: ascamera_hp60c_color_0
3. Añadir la Nube de Puntos
   - Botón Add - Pestaña By Topic
   - Seleccionar: /ascamera_hp60c/camera_publisher/depth0/points
   - Tipo: PointCloud2
4. Configuración en Caso de Error (QoS)

   - En el panel izquierdo, despliega las opciones de PointCloud2 y ajusta:
     - Reliability Policy: Best Effort (vital para evitar lag)
     - Durability Policy: Volatile
     - Style: Points
     - Color Transformer: AxisColor (colorea por profundidad)

## 🧩 Notas

- Verifica que las librerías propietarias correspondan a tu arquitectura.
- Paquete diseñado para ROS 2.
