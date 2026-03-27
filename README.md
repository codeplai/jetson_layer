# MINEBOT-Q — Jetson Orin NX Layer

Capa de visión, LiDAR y control del Go2 para inspección minera subterránea.

## Arquitectura de nodos y tópicos

```
  QRB2210 (192.168.12.10)              Jetson Orin NX (192.168.12.1)
 ┌──────────────────────┐          ┌──────────────────────────────────────┐
 │  gas_publisher_node  │          │                                      │
 │  ├─ /gas/mq4    ─────┼── DDS ──┼──►┐                                  │
 │  ├─ /gas/mq7    ─────┼── DDS ──┼──►│  go2_bridge_node                 │
 │  ├─ /gas/mq135  ─────┼── DDS ──┼──►│  (Dual Confirmation)             │
 │  └─ /gas/status ─────┼── DDS ──┼──►│  ├─► /go2/cmd_vel                │
 └──────────────────────┘          │   │  ├─► /go2/behavior               │
                                   │   │  └─► /go2/alert                  │
                                   │   │       ▲     ▲                    │
                                   │   │       │     │                    │
                                   │  camera_node    lidar_node           │
                                   │  ├─ /camera/image_raw   /lidar/points│
                                   │  ├─ /camera/detections  /lidar/scan2d│
                                   │  └─ /camera/compressed  /lidar/doppler│
                                   │                                      │
                                   │  state_publisher_node                │
                                   │  ├─ /go2/state                       │
                                   │  ├─ /go2/pose                        │
                                   │  └─ /go2/battery                     │
                                   │                                      │
                                   │  rosbridge_websocket (:9090)  ───────┼──► Dashboard Web
                                   └──────────────────────────────────────┘
                                                                        192.168.12.20
                                                                        Laptop operador
```

## Red del sistema

| IP | Dispositivo | Rol |
|----|-------------|-----|
| `192.168.12.1` | Unitree Go2 / Jetson Orin NX | Visión, LiDAR, control, rosbridge :9090 |
| `192.168.12.10` | Arduino UNO Q / QRB2210 | Sensores de gases + IMU |
| `192.168.12.20` | Laptop operador | Dashboard web |

## Requisitos

- JetPack 5.x (Ubuntu 20.04) en Jetson Orin NX
- ROS2 Humble instalado nativo
- Python 3.10

```bash
# Dependencias ROS2
sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-rosbridge-server \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-cv-bridge -y

# Dependencias Python
pip install -r requirements.txt
```

## Convertir YOLOv8n a ONNX INT8

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=640
# Copiar el .onnx a /home/unitree/minebot_ws/models/
```

## Build

```bash
cd ~/minebot_ws
colcon build --packages-select jetson_layer
source install/setup.bash
```

## Ejecución

```bash
# Lanzar toda la capa Jetson
ros2 launch jetson_layer jetson_layer.launch.py

# Modo simulación (sin hardware)
ros2 launch jetson_layer jetson_layer.launch.py sim:=true

# Con log detallado
ros2 launch jetson_layer jetson_layer.launch.py log_level:=DEBUG
```

## Verificar tópicos cruzados QRB2210 ↔ Jetson

```bash
# Desde el Jetson — debe mostrar datos del QRB2210
ros2 topic echo /gas/mq4
ros2 topic hz /gas/mq4     # esperado: ~10 Hz

# Listar todos los tópicos del sistema
ros2 topic list
```

Ambos dispositivos deben tener:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

## Instalar como servicio systemd

```bash
sudo cp systemd/minebot-jetson.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable minebot-jetson
sudo systemctl start minebot-jetson

# Verificar
sudo systemctl status minebot-jetson
journalctl -u minebot-jetson -f
```

## Tests

```bash
cd ~/minebot_ws/src/jetson_layer
pytest test/test_dual_confirmation.py -v
```

## Dual Confirmation Architecture

```
ALERTA CONFIRMADA = (gas_alert)
                    AND (camera_obstacle_detected OR lidar_obstacle_near)

Precision: 78% (sensor unico) → 96% (dual confirmation)
Timeout de confirmacion: 2 segundos
```

| Escenario | Accion |
|-----------|--------|
| CRITICAL + confirmado | `stop()` + `retreat(2.0m)` |
| WARNING + confirmado | velocidad al 50% + `scan_zone()` |
| SAFE | resume velocidad normal |
