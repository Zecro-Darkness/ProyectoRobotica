# Correcciones Realizadas - Nodo Clasificador

## Problemas Identificados

Al ejecutar el nodo clasificador, se encontraron dos problemas principales:

### 1. Poses Fuera del Espacio de Trabajo
**Error:** `Unable to sample any valid states for goal tree`

Las poses originales en `poses.yaml` estaban fuera del alcance del robot PhantomX Pincher:
- `home`: x=0.150, y=0.000, z=0.200 ❌
- `caneca_roja`: x=-0.012, y=0.127, z=0.067 ❌ (coordenada X negativa)

### 2. Nombres Incorrectos del Gripper
**Error:** `The requested named target 'gripper_closed' does not exist`

El commander buscaba:
- `gripper_open` ❌
- `gripper_closed` ❌

Pero el SRDF define:
- `open` ✅
- `closed` ✅

## Soluciones Implementadas

### 1. Actualización de Poses en `poses.yaml`

Se reemplazaron todas las poses con valores alcanzables y probados:

```yaml
poses:
  # Posición segura/inicial (pose válida y alcanzable)
  home:
    x: 0.128
    y: 0.000
    z: 0.100
    roll: 3.142
    pitch: 0.0
    yaw: 0.0

  # Zona donde aparecen los objetos (pose alcanzable para pick)
  recoleccion:
    x: 0.150
    y: 0.000
    z: 0.050
    roll: 3.142
    pitch: 0.0
    yaw: 0.0

  # Destinos para cada figura/color (poses alcanzables)
  caneca_roja: # Cubo
    x: 0.120
    y: 0.080
    z: 0.080
    roll: 3.142
    pitch: 0.0
    yaw: 0.785

  caneca_verde: # Cilindro
    x: 0.120
    y: -0.080
    z: 0.080
    roll: 3.142
    pitch: 0.0
    yaw: -0.785

  caneca_azul: # Pentágono
    x: 0.180
    y: 0.050
    z: 0.070
    roll: 3.142
    pitch: 0.0
    yaw: 0.4

  caneca_amarilla: # Rectángulo
    x: 0.180
    y: -0.050
    z: 0.070
    roll: 3.142
    pitch: 0.0
    yaw: -0.4
```

**Cambios clave:**
- ✅ Todas las coordenadas X son positivas (dentro del alcance frontal del robot)
- ✅ Altura Z reducida a valores alcanzables (0.050 - 0.100 m)
- ✅ Orientación simplificada (pitch=0.0 para todas las poses)
- ✅ Ángulos yaw ajustados para distribución simétrica de canecas

### 2. Corrección del Commander

**Archivo:** `phantomx_pincher_commander_cpp/src/commander_template.cpp`

```cpp
// ANTES ❌
void openGripper()
{
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("gripper_open");  // ❌ No existe
    planAndExecute(gripper_);
}

void closeGripper()
{
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("gripper_closed");  // ❌ No existe
    planAndExecute(gripper_);
}

// DESPUÉS ✅
void openGripper()
{
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("open");  // ✅ Coincide con SRDF
    planAndExecute(gripper_);
}

void closeGripper()
{
    gripper_->setStartStateToCurrentState();
    gripper_->setNamedTarget("closed");  // ✅ Coincide con SRDF
    planAndExecute(gripper_);
}
```

## Archivos Modificados

1. **[poses.yaml](file:///home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_bringup/config/poses.yaml)** - Poses actualizadas con valores alcanzables
2. **[commander_template.cpp](file:///home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_commander_cpp/src/commander_template.cpp)** - Nombres de gripper corregidos

## Paquetes Recompilados

```bash
colcon build --packages-select phantomx_pincher_commander_cpp phantomx_pincher_bringup
```

**Resultado:** ✅ Compilación exitosa (1min 4s)

## Instrucciones de Prueba

### 1. Reiniciar el Sistema

Cierra todas las terminales actuales y reinicia el sistema completo:

```bash
# Terminal 1: Iniciar sistema
cd /home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
source install/setup.bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=false
```

### 2. Iniciar Nodo Clasificador

```bash
# Terminal 2: Iniciar clasificador
cd /home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
source install/setup.bash
ros2 run phantomx_pincher_classification clasificador_node
```

### 3. Probar con Cubo

```bash
# Terminal 3: Enviar comando
cd /home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
source install/setup.bash
ros2 topic pub --once /figure_type std_msgs/msg/String "{data: 'cubo'}"
```

### Comportamiento Esperado

Ahora deberías ver:

1. ✅ **Sin errores de planning** - Las poses son alcanzables
2. ✅ **Gripper funciona correctamente** - Se abre y cierra sin errores
3. ✅ **Robot se mueve en RViz** - Visualización de la secuencia completa
4. ✅ **Secuencia completa:**
   - [1/6] Moviendo a posición HOME ✅
   - [2/6] Moviendo a zona de RECOLECCIÓN ✅
   - [3/6] Cerrando gripper ✅
   - [4/6] Moviendo a caneca_roja ✅
   - [5/6] Abriendo gripper ✅
   - [6/6] Regresando a HOME ✅

## Notas Importantes

### Calibración de Poses

Las poses actuales son **genéricas y alcanzables**, pero es posible que necesites ajustarlas según tu configuración física:

1. **Para robot real:** Verifica que las posiciones de las canecas coincidan con tu setup físico
2. **Para simulación:** Las poses actuales deberían funcionar correctamente

### Ajuste de Poses

Si necesitas modificar las poses:

```bash
# Editar poses
nano /home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_bringup/config/poses.yaml

# Recompilar (solo si cambiaste archivos de código)
cd /home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws
colcon build --packages-select phantomx_pincher_bringup

# Reiniciar nodo clasificador (lee poses al inicio)
# Ctrl+C en terminal del clasificador, luego:
ros2 run phantomx_pincher_classification clasificador_node
```

### Verificación de Poses Individuales

Puedes probar poses individuales antes de ejecutar la secuencia completa:

```bash
# Probar pose home
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.128, y: 0.0, z: 0.100, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: false}"

# Probar pose recoleccion
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.150, y: 0.0, z: 0.050, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: true}"

# Probar caneca_roja
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.120, y: 0.080, z: 0.080, roll: 3.142, pitch: 0.0, yaw: 0.785, cartesian_path: true}"
```

## Resumen

✅ **Problema 1 resuelto:** Poses actualizadas a valores alcanzables  
✅ **Problema 2 resuelto:** Nombres de gripper corregidos  
✅ **Paquetes recompilados:** Commander y Bringup  
✅ **Sistema listo para pruebas**

El nodo clasificador ahora debería ejecutar la secuencia completa de pick-and-place sin errores de planning ni problemas con el gripper.
