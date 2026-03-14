# ASCamera ROS 2 Driver

Driver para cámaras RGB-D de Angstrong que publica imágenes RGB, mapas de profundidad y nubes de puntos 3D en ROS 2.

---

## 📋 Requisitos

- ROS 2 (probado en Jazzy)
- Arquitectura:
  - x86_64 (PC / Laptop)
  - aarch64 (Raspberry Pi / ARM)

---

## 📦 Estructura del Paquete

```
ascamera/
├── launch/                     # Archivos de lanzamiento
├── configurationfiles/         # Configuraciones específicas por modelo
├── libs/                       # Librerías propietarias del SDK
│   └── lib/
│       ├── x86_64-linux-gnu/   # Para PC
│       └── aarch64-linux-gnu/  # Para Raspberry Pi / ARM
├── include/                    # Headers del paquete
└── src/                        # Código fuente
```

---

# ⚙️ Instalación

## 1️⃣ Configuración de Librerías (OBLIGATORIO)

Las librerías propietarias deben instalarse en el sistema antes de compilar.

### Para PC (x86_64)

```bash
cd ~/ROS/camara_ws/src/ascamera/libs/lib/x86_64-linux-gnu/
sudo cp *.so /usr/lib/
sudo ldconfig
```

### Para Raspberry Pi / ARM (aarch64)

```bash
cd ~/ROS/camara_ws/src/ascamera/libs/lib/aarch64-linux-gnu/
sudo cp *.so /usr/lib/
sudo ldconfig
```

---

## 2️⃣ Reglas USB (udev)

Para usar la cámara sin permisos de superusuario:

```bash
cd ~/ROS/camara_ws/src/ascamera/scripts
sudo bash create_udev_rules.sh
```

Desconecta y vuelve a conectar la cámara después de ejecutar el script.

---

## 3️⃣ Compilación

```bash
cd ~/ROS/camara_ws
colcon build --packages-select ascamera
source install/setup.bash
```
# Después de compilar, configura las librerías (solo necesario una vez)
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/ROS/camara_ws/install/ascamera/lib/' >> ~/.bashrc
source ~/.bashrc

# Para ejecutar la cámara HP60C (modelo detectado)
ros2 launch ascamera hp60c.launch.py

# Nota: El launch específico del modelo incluye parámetros adicionales
# que evitan el error "stack smashing detected"
---

# 🚀 Uso

## Ejecutar el nodo de la cámara

### Modelo HP60C

```bash
ros2 launch ascamera hp60c.launch.py
```

### Modelo genérico (detección automática)

```bash
ros2 launch ascamera ascamera.launch.py
```

---

# 📡 Tópicos Publicados

| Tópico | Tipo | Descripción |
|--------|------|------------|
| `/ascamera/ascamera_node/rgb0/image` | `sensor_msgs/Image` | Imagen RGB |
| `/ascamera/ascamera_node/depth0/image_raw` | `sensor_msgs/Image` | Mapa de profundidad (16UC1) |
| `/ascamera/ascamera_node/depth0/points` | `sensor_msgs/PointCloud2` | Nube de puntos 3D |
| `/ascamera/ascamera_node/depth0/camera_info` | `sensor_msgs/CameraInfo` | Calibración profundidad |
| `/ascamera/ascamera_node/rgb0/camera_info` | `sensor_msgs/CameraInfo` | Calibración RGB |

---

## 🔍 Verificar Publicación de Datos

### Ver tópicos activos

```bash
ros2 topic list
```

### Ver frecuencia de publicación

```bash
ros2 topic hz /ascamera/ascamera_node/rgb0/image
```

### Ver tipo de mensaje

```bash
ros2 topic type /ascamera/ascamera_node/depth0/points
```

---

# 🖥️ Visualización en RViz2

## 1️⃣ Iniciar RViz2

```bash
rviz2
```

## 2️⃣ Configuración Global

- En el panel izquierdo → **Fixed Frame**
- Seleccionar: `ascamera_color_0`

## 3️⃣ Ver imagen RGB

- Botón **Add**
- Seleccionar **Image**
- En **Image Topic** elegir:
  ```
  /ascamera/ascamera_node/rgb0/image
  ```

## 4️⃣ Ver nube de puntos

- Botón **Add**
- Seleccionar **PointCloud2**
- En **Topic** elegir:
  ```
  /ascamera/ascamera_node/depth0/points
  ```

### Ajustes recomendados para la nube de puntos

- **Style**: Points
- **Size (m)**: 0.01
- **Color Transformer**: AxisColor
- **Reliability Policy**: Best Effort

---

# 💾 Grabación de Rosbag

## Grabar datos

```bash
ros2 bag record -o camara_dataset \
  /ascamera/ascamera_node/rgb0/image \
  /ascamera/ascamera_node/depth0/image_raw \
  /ascamera/ascamera_node/depth0/points
```

## Reproducir datos

```bash
ros2 bag play camara_dataset
```

---

# 🔧 Solución de Problemas

## ❌ Error: libAngstrongCameraSdk.so: cannot open shared object file

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ROS/camara_ws/install/ascamera/lib:~/ROS/camara_ws/libs/lib/x86_64-linux-gnu/
```

Agregar la línea anterior al `~/.bashrc` para hacerlo permanente.

---

## ❌ Error: "config file path error"

Verifica que el parámetro se llame EXACTAMENTE `confiPath`:

```bash
ros2 param get /ascamera/ascamera_node confiPath
```

⚠️ Debe ser `confiPath` (no `configPath`).

---

## ❌ Error de permisos USB

```bash
ls -la /dev/video*
```

Si no tienes permisos:

```bash
sudo bash ~/ROS/camara_ws/src/ascamera/scripts/create_udev_rules.sh
```

---

## ❌ La nube de puntos no se ve en RViz2

Verificar publicación:

```bash
ros2 topic echo --once /ascamera/ascamera_node/depth0/points | head -20
```

En RViz2 asegurarse de:
- Style = Points
- Size > 0

---

# 📝 Notas Importantes

- Las librerías propietarias son específicas para cada arquitectura (x86_64 vs aarch64).
- El parámetro del launch file debe llamarse `confiPath`.
- Para múltiples cámaras, cada una publica en su propio namespace.
- Compatible con ROS 2 Humble, Iron y Jazzy.

---

# 📄 Licencia

GPLv2 — Ver archivo `LICENSE` para más detalles.

---

# 👥 Mantenedores

- Joyo — luzhuoyue@angstrong.com

---

# 🤝 Contribuciones

Para reportar bugs o solicitar nuevas funcionalidades:
- Contactar al mantenedor.
- Crear un issue en el repositorio.

---
