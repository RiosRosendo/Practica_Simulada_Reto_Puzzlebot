# Puzzlebot Fully Autonomous Navigation

Sistema de navegación autónoma completo con ROS 2 Humble para el robot diferencial Puzzlebot. El robot construye su propio mapa con SLAM, planifica rutas libres de colisión con A\*, las sigue con un controlador de persecución pura y evita obstáculos en tiempo real.

> Curso: Integración de robótica y sistemas inteligentes — Tecnológico de Monterrey

---

## Arquitectura del sistema

```
Cámara + LiDAR (Gazebo)
        │
        ▼
   Sensor Bridge (ros_gz_bridge)
  /scan  /camera/image_raw  /imu
        │
        ▼
  ┌─────────────┐        ┌──────────────────┐
  │  SLAM Node  │──/map──▶  A* Planner Node │◀── /goal_pose  (usuario)
  │ (slam_node) │        └──────────────────┘
  └─────────────┘                 │ /path
        │ /slam_pose              ▼
        │              ┌─────────────────────┐
        └─────────────▶│  Path Follower Node │◀── /scan (evasión)
                       └─────────────────────┘
                                 │ /cmd_vel (Twist)
                                 ▼
                          twist_relay node
                                 │ /puzzlebot_controller/cmd_vel (TwistStamped)
                                 ▼
                        simple_controller node
                                 │ wheel velocity commands
                                 ▼
                          Gazebo Simulation
```

### Árbol TF

```
map ──▶ odom ──▶ base_footprint ──▶ base_link ──▶ lidar_link
                                               ──▶ camera_link
                                               ──▶ wheel_right_link
                                               ──▶ wheel_left_link
```

---

## Paquetes ROS 2

| Paquete | Descripción |
|---|---|
| `puzzlebot_description` | URDF/Xacro, mundos Gazebo, scripts de edición de mapas |
| `puzzlebot_controller` | `simple_controller` (odometría), `twist_relay` |
| `puzzlebot_slam` | SLAM personalizado con cuadrícula de ocupación |
| `puzzlebot_navigation` | Planificador A\*, seguidor pure-pursuit, spawner de obstáculos |

---

## Requisitos

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-rviz2 \
  python3-scipy

pip install scipy numpy
```

---

## Build

```bash
cd ~/puzzlebot_nav_ws
colcon build
source install/setup.bash
```

> Ejecuta `source install/setup.bash` en **cada terminal nueva** antes de cualquier comando ROS 2.

---

## Inicio rápido — 2 terminales

### Terminal 1 — Lanzar todo el sistema

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 launch puzzlebot_navigation autonomous_nav.launch.py
```

Espera ~30-60 segundos hasta ver `"Map published"` en la terminal. Cuando aparezca el mapa en RViz, el robot habrá completado su giro inicial de 360° y estará listo para recibir objetivos.

**Componentes que inicia:**

| Componente | Función |
|---|---|
| Gazebo | Simulación con robot, LiDAR, cámara, IMU |
| `simple_controller` + `twist_relay` | Convierte `/cmd_vel` → comandos de ruedas |
| `slam_node` | Construye el mapa en tiempo real; publica `/map` y `/slam_pose` |
| `astar_planner` | Lee `/map` + `/goal_pose`, planifica con A\* y publica `/path` |
| `path_follower` | Sigue `/path` con pure-pursuit; giro inicial 360° al arrancar |
| `obstacle_spawner` | Añade una caja aleatoria a Gazebo cada 20 s |
| RViz2 | Visualiza mapa, ruta, modelo del robot, scan LiDAR |

### Terminal 2 — Enviar objetivo de navegación

**Opción A — click en RViz (recomendado):**
En la barra de herramientas de RViz, selecciona el botón **"2D Goal Pose"** (flecha verde). Haz clic en el mapa donde quieres que vaya el robot y arrastra para darle orientación.

**Opción B — comando de terminal:**
```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}'
```

Cambia `x` e `y` al destino deseado (en metros, relativo al punto de inicio del robot).

---

## Mundos disponibles

| Mundo | Descripción | Comando |
|---|---|---|
| `warehouse` | Arena rectangular con racks (configurable con el editor) | `world_name:=warehouse` |
| `obstacles` | Paredes + cajas + cilindro + pared diagonal | `world_name:=obstacles` |
| `empty` | Piso plano sin obstáculos | `world_name:=empty` |

```bash
# Mundo warehouse con racks (sin spawner dinámico)
ros2 launch puzzlebot_navigation autonomous_nav.launch.py \
  world_name:=warehouse spawn_obstacles:=false

# Mundo con obstáculos y spawner dinámico
ros2 launch puzzlebot_navigation autonomous_nav.launch.py \
  world_name:=obstacles

# Sin spawner dinámico
ros2 launch puzzlebot_navigation autonomous_nav.launch.py \
  world_name:=obstacles spawn_obstacles:=false
```

---

## Editor visual de mapas (World Editor)

Herramienta con sliders para diseñar el arena y colocar los racks antes de lanzar la simulación. Muestra un preview 2D en tiempo real.

### Abrir el editor

```bash
python3 ~/puzzlebot_nav_ws/src/Puzzlebot-Challenge-/src/puzzlebot_description/scripts/world_editor.py
```

### Controles

| Slider | Efecto |
|---|---|
| **Ancho X / Largo Y** | Tamaño del arena rectangular |
| **X / Y** (por rack) | Posición del rack en metros (0,0 = origen del robot) |
| **Rotación °** | Orientación del rack (0° = largo en Y, 90° = largo en X) |
| **Tamaño** | Escala del rack (1.0 = tamaño original) |
| **+ Agregar Rack** | Añade un rack adicional |
| **− Quitar último** | Elimina el último rack |
| **💾 Guardar** | Escribe `warehouse.world` |

### Flujo de trabajo

1. Abre el editor y ajusta el arena y racks con los sliders
2. El preview 2D se actualiza en tiempo real (robot = punto verde, racks = polígonos de colores)
3. Haz clic en **"Guardar warehouse.world"**
4. Reconstruye y relanza:

```bash
cd ~/puzzlebot_nav_ws
colcon build --packages-select puzzlebot_description
source install/setup.bash
ros2 launch puzzlebot_navigation autonomous_nav.launch.py \
  world_name:=warehouse spawn_obstacles:=false
```

### Edición manual del world file

También puedes editar directamente `src/puzzlebot_description/worlds/warehouse.world`. Cada rack tiene una línea `<pose>`:

```xml
<pose>x  y  0  0  0  yaw</pose>
```

- `x`, `y` = posición en metros
- `yaw` = rotación en radianes (`0` = largo en Y, `1.5708` = largo en X, `3.1416` = 180°)

---

## Diagnóstico y debug

### Verificar que el pipeline completo está activo

```bash
# ¿El LiDAR de Gazebo está publicando?
ros2 topic hz /scan

# ¿El SLAM está publicando el mapa?
ros2 topic hz /map

# ¿SLAM está estimando la pose?
ros2 topic hz /slam_pose

# ¿El simple_controller está publicando odometría?
ros2 topic hz /puzzlebot_controller/odom

# ¿El path follower está enviando velocidad?
ros2 topic echo /cmd_vel
```

### Verificar nodos activos

```bash
ros2 node list
# Debe incluir: /slam_node /astar_planner /path_follower
#               /simple_controller /twist_relay /obstacle_spawner
```

### Prueba directa del motor (sin planner)

```bash
# Mover el robot hacia adelante directamente
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}}' --rate 10
```

Si el robot no se mueve con este comando, los controladores aún no han spawneado (espera más tiempo tras el launch).

### Árbol TF

```bash
ros2 run tf2_tools view_frames
```

### Grafo de nodos

```bash
ros2 run rqt_graph rqt_graph
```

---

## Ajuste de parámetros de navegación

Edita `src/puzzlebot_navigation/config/nav_params.yaml` y reconstruye:

| Parámetro | Default | Efecto |
|---|---|---|
| `inflation_radius` | 0.25 m | Radio de inflado de obstáculos en A\* |
| `obstacle_threshold` | 50 | Umbral de ocupación para tratar celda como obstáculo |
| `lookahead_distance` | 0.40 m | Distancia al punto objetivo en pure-pursuit |
| `linear_speed` | 0.20 m/s | Velocidad lineal de navegación |
| `angular_gain` | 2.0 | Ganancia proporcional de giro |
| `goal_tolerance` | 0.15 m | Distancia para considerar el objetivo alcanzado |
| `obstacle_stop_dist` | 0.35 m | Distancia de parada de emergencia |
| `obstacle_slow_dist` | 0.60 m | Distancia para comenzar a reducir velocidad |
| `initial_spin_duration` | 9.0 s | Duración del giro inicial de mapeo (0 = desactivado) |
| `initial_spin_speed` | 0.70 rad/s | Velocidad del giro inicial |
| `spawn_interval` | 20 s | Segundos entre spawns de obstáculos dinámicos |

Ajuste de parámetros SLAM en `src/puzzlebot_slam/config/slam_params.yaml`:

| Parámetro | Default | Efecto |
|---|---|---|
| `use_icp` | false | ICP desactivado (usar solo odometría en simulación) |
| `l_occ` | 0.70 | Incremento log-odds por impacto LiDAR |
| `l_free` | -0.12 | Decremento log-odds por rayo libre |
| `display_l_occ` | 1.0 | Umbral para mostrar celda como ocupada en RViz |
| `map_publish_every` | 5 | Publicar mapa cada N scans |

---

## Referencia de tópicos

| Tópico | Tipo | Publicador | Suscriptores |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge | SLAM, A\*, path follower |
| `/puzzlebot_controller/odom` | `nav_msgs/Odometry` | simple_controller | SLAM |
| `/map` | `nav_msgs/OccupancyGrid` | slam_node | A\* planner |
| `/slam_pose` | `geometry_msgs/PoseStamped` | slam_node | A\*, path follower |
| `/goal_pose` | `geometry_msgs/PoseStamped` | usuario / RViz | A\* planner |
| `/path` | `nav_msgs/Path` | astar_planner | path follower |
| `/cmd_vel` | `geometry_msgs/Twist` | path_follower | twist_relay |
| `/puzzlebot_controller/cmd_vel` | `geometry_msgs/TwistStamped` | twist_relay | simple_controller |
