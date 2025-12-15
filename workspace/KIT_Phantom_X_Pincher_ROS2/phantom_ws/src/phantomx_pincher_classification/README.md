# Guía de Uso: Nodo de Clasificación PhantomX Pincher

## Descripción General

El nodo `clasificador_node` orquesta operaciones automáticas de pick-and-place para el robot PhantomX Pincher basándose en el tipo de figura detectada.

## Mapeo de Figuras a Canecas

| Figura | Caneca | Color |
|--------|--------|-------|
| cubo | caneca_roja | Rojo |
| cilindro | caneca_verde | Verde |
| pentagono | caneca_azul | Azul |
| rectangulo | caneca_amarilla | Amarillo |

## Instalación y Compilación

```bash
cd /home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
colcon build --packages-select phantomx_pincher_classification
source install/setup.bash
```

## Uso Básico

### Opción 1: Iniciar el nodo directamente

```bash
# Terminal 1: Iniciar el sistema completo (simulación)
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=false

# Terminal 2: Iniciar el nodo de clasificación
ros2 run phantomx_pincher_classification clasificador_node

# Terminal 3: Enviar comando de clasificación
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'cubo'}"
```

### Opción 2: Usar el launch file

```bash
# Terminal 1: Iniciar el sistema completo
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=false

# Terminal 2: Iniciar clasificador
ros2 launch phantomx_pincher_classification clasificador.launch.py

# Terminal 3: Enviar comando de clasificación
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'cilindro'}"
```

## Comandos de Prueba

### Probar todas las figuras

```bash
# Cubo → Caneca Roja
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'cubo'}"

# Cilindro → Caneca Verde
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'cilindro'}"

# Pentágono → Caneca Azul
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'pentagono'}"

# Rectángulo → Caneca Amarilla
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'rectangulo'}"
```

## Secuencia de Operación

Cuando se recibe un tipo de figura, el nodo ejecuta la siguiente secuencia:

1. **[1/6] Mover a HOME** - Posición segura inicial
2. **[2/6] Mover a RECOLECCIÓN** - Zona donde aparecen los objetos
3. **[3/6] Cerrar Gripper** - Recoger el objeto
4. **[4/6] Mover a CANECA** - Ir a la caneca correspondiente según el tipo de figura
5. **[5/6] Abrir Gripper** - Soltar el objeto
6. **[6/6] Regresar a HOME** - Volver a posición segura

## Tópicos Utilizados

### Suscripciones
- `/figure_type` (`std_msgs/msg/String`) - Tipo de figura a clasificar

### Publicaciones
- `/pose_command` (`phantomx_pincher_interfaces/msg/PoseCommand`) - Comandos de posición para el brazo
- `/open_gripper` (`example_interfaces/msg/Bool`) - Control del gripper (true=abrir, false=cerrar)

## Monitoreo

### Ver comandos de pose publicados
```bash
ros2 topic echo /pose_command
```

### Ver comandos del gripper
```bash
ros2 topic echo /open_gripper
```

### Ver logs del nodo
```bash
ros2 node info /clasificador_node
```

## Configuración de Poses

Las poses se cargan automáticamente desde:
```
/home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_bringup/config/poses.yaml
```

Para modificar las posiciones, edita este archivo y reinicia el nodo.

## Solución de Problemas

### El nodo no encuentra las poses
- Verifica que el paquete `phantomx_pincher_bringup` esté compilado
- Asegúrate de haber ejecutado `source install/setup.bash`

### El robot no se mueve
- Verifica que el nodo `commander` esté ejecutándose
- Revisa que MoveIt esté correctamente configurado
- Comprueba los logs del nodo clasificador

### Tipo de figura no reconocido
- Los tipos válidos son: `cubo`, `cilindro`, `pentagono`, `rectangulo` (todo en minúsculas)
- El nodo es case-insensitive, pero evita espacios extra

## Uso con Robot Real

```bash
# Iniciar con robot real
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true

# Iniciar clasificador
ros2 run phantomx_pincher_classification clasificador_node

# Enviar comando
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'cubo'}"
```

**IMPORTANTE:** Con el robot real, asegúrate de que:
- Los servos estén correctamente conectados y energizados
- El baudrate esté configurado correctamente (57600)
- Haya espacio libre para el movimiento del robot
- Las poses en `poses.yaml` estén calibradas para tu configuración física
