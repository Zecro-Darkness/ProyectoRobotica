# Comandos de Operación (Prueba Manual)

**Comando para iniciar la secuencia (Faltante):**
Para que el robot reaccione a los comandos, primero debes iniciar el nodo clasificador:

```bash
ros2 run phantomx_pincher_classification clasificador_node
```

Para verificar el funcionamiento de cada rutina sin necesidad de la cámara, se deben ejecutar los siguientes comandos en una terminal con el entorno ROS2 cargado:

### Clasificar CUBO (Caneca Roja):

```bash
ros2 topic pub /figure_type std_msgs/msg/String "data: 'cubo'" --once
```

### Clasificar CILINDRO (Caneca Verde):

```bash
ros2 topic pub /figure_type std_msgs/msg/String "data: 'cilindro'" --once
```

### Clasificar PENTÁGONO (Caneca Azul):

```bash
ros2 topic pub /figure_type std_msgs/msg/String "data: 'pentagono'" --once
```

### Clasificar RECTÁNGULO (Caneca Amarilla):

```bash
ros2 topic pub /figure_type std_msgs/msg/String "data: 'rectangulo'" --once
```
