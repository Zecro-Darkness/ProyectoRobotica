# Manipulación y Clasificación Robot PhantomX Pincher

## Integrantes

- Alejandro Mendivelso Torres
- Juan Manuel Beltran Botello 
- Oscar Jhondairo Siabato Leon
- Edgar Esteban Erazo Lagos


## Conceptos Técnicos Clave
Para entender y modificar este proyecto, es útil tener en cuenta los siguientes conceptos: 
- Máquina de Estados (State Machine)
El software que gestiona la secuencia de clasificación no opera como un simple script lineal (ejecuta A, luego B, luego C). En cambio, está estructurado como una Máquina de Estados (State Machine).

¿Por qué es crucial esta arquitectura? En robótica, las operaciones físicas no son instantáneas: un movimiento puede demorar 3 segundos, y la apertura de una pinza 1 segundo. El programa debe gestionar estas esperas sin bloquearse.

El corazón del sistema sabe con precisión en qué "Estado" se encuentra actualmente (por ejemplo: STATE_GRIPPING_CLOSING). Un temporizador interno verifica si el tiempo asignado a ese estado ha transcurrido. Una vez cumplido el plazo, el sistema transiciona automáticamente al siguiente "Estado" lógico de la secuencia (por ejemplo: STATE_LIFTING_CUBE). Este enfoque garantiza un control robusto, asíncrono y evita interrupciones o congelamientos del programa.
- **Cinemática Directa vs Inversa:**
Para mover el brazo, se utilizan dos enfoques fundamentales de cinemática:Cinemática Directa (Forward Kinematics): En el modo de teleoperación (Teleop), el usuario está aplicando conceptualmente cinemática directa. Se envían comandos para controlar directamente los ángulos de cada articulación ($q_1, q_2, q_3...$). El sistema calcula cuál es la posición final ($X, Y, Z$) resultante. Es un método intrínsecamente seguro ya que el comando es la posición del motor.Cinemática Inversa (Inverse Kinematics - IK): El clasificador automático utiliza IK. El comando es la posición deseada en el espacio de trabajo: "Mueve el efector final a $X=0.20$m, $Y=0.10$m". El algoritmo debe resolver internamente los ángulos ($q_i$) necesarios para alcanzar ese punto. Si la coordenada de destino está fuera del alcance físico (workspace) del robot, el sistema de IK detectará que no hay una solución válida, reportará un error e interrumpirá la tarea para prevenir daños.
- Comunicación ROS2
El proyecto implementa los dos principales mecanismos de comunicación inter-proceso de ROS 2:

- **Topics (Tópicos):** Son ideales para la transmisión de datos rápida, continua y unidireccional (modelo Publicar/Suscribir). Se utilizan, por ejemplo, cuando un sensor o un usuario externo publica el tipo de objeto detectado (/figure_type) al que el nodo clasificador está suscrito para iniciar su tarea.

- **Actions (Acciones):** Están diseñadas para manejar tareas de larga duración con la necesidad de feedback y cancelación (modelo Cliente/Servidor). Por ejemplo, cuando el nodo clasificador necesita que el robot se mueva, invoca una acción (típicamente FollowJointTrajectory). El cliente de acción le permite al clasificador esperar de manera controlada y no bloqueante hasta que el movimiento del brazo se haya completado exitosamente antes de proceder con el siguiente paso lógico de la Máquina de Estados.

## 1. Resumen del Proyecto
Este proyecto implementa un sistema avanzado de control y automatización para el brazo robótico PhantomX Pincher utilizando ROS2 (Robot Operating System 2). El sistema está diseñado para realizar tareas de manipulación de objetos ("pick and place") de manera robusta y precisa.

El desarrollo abarca dos modos principales de operación:

-**Teleoperación Manual:** Permite a un operador controlar cada articulación del robot, el efector final (gripper) y herramientas externas (bomba de vacío) utilizando el teclado del ordenador.
-**Clasificación Automática:** Un sistema autónomo que recibe un comando específico indicando el tipo de objeto ("cubo", "cilindro", "pentágono", "rectángulo") y ejecuta una coreografía de movimientos precisa para recoger el objeto y depositarlo en su contenedor específico.

## 2. Descripción General del Proyecto

Este proyecto implementa una solución de automatización para tareas de "pick and place" utilizando un brazo robótico PhantomX Pincher. El sistema utiliza ROS2 sobre Ubuntu 20.04/Linux para controlar el robot de manera precisa.

### Herramientas y Piezas Utilizadas

- **Robot:** PhantomX Pincher (4 Grados de Libertad + Gripper).
- **Software Principal:** ROS2 (Robot Operating System 2).
- **Lenguajes:**
  - Python (Lógica de control)
  - XML/Xacro (Descripción del robot)
  - YAML (Configuración)
  -  
### 2.1 Objetivo General

Implementar un sistema de control y automatización para el brazo robótico PhantomX Pincher usando ROS2.

### 2.2 Objetivos Específicos

Desarrollar un nodo de teleoperación articular.

Implementar un sistema automático de clasificación de objetos.

Integrar hardware externo (bomba de vacío).

Garantizar movimientos seguros y repetibles mediante estados y posiciones predefinidas.

### 2.3 Modos de Operación

- Teleoperación manual
- Clasificación automática

## Parte 1

### Objetivo
El objetivo es crear una rutina de robot que pueda recoger 4 objetos distintos que siempre se ubicarán en la zona de recolección y luego depositarlos en la caneca correspondiente según su tipo.

Los objetos son:

- Un cubo
- Un cilindro
- Un pentágono
- Un rectángulo
Nota: Todos estos objetos tendrán la misma altura.

La clasificación de los objetos en las canecas será:

- El cubo debe dejarse en la caneca roja
- El cilindro debe dejarse en la caneca verde
- El pentágono debe dejarse en la caneca azul
- El rectángulo debe dejarse en la caneca amarilla

## 1. Implementación en Código
La lógica se ha centralizado en el nodo 
clasificador_node.py. Se utiliza un diccionario para mapear cada figura entrante con su destino (caneca) correspondiente, cumpliendo estrictamente con los colores solicitados.

´´´ python
# Extracto de clasificador_node.py
self.figure_to_bin = {
    'cubo': 'caneca_roja',            # Cubo -> Caneca Roja
    'cilindro': 'caneca_verde',       # Cilindro -> Caneca Verde
    'pentagono': 'caneca_azul',       # Pentágono -> Caneca Azul
    'rectangulo': 'caneca_amarilla'   # Rectángulo -> Caneca Amarilla
}
´´´

## 2. Lógica de Deposición (Secuencia Completa)

El robot no solo mueve el brazo, sino que ejecuta una **Máquina de Estados** para que la secuencia de *movimiento + agarre* sea robusta:

1. **Recepción del comando:** el nodo escucha el tópico `/figure_type`.
2. **Preparación:** mueve el robot a `HOME2` y abre el gripper.
3. **Recolección:** baja a la posición de recolección (`recoleccion`) definida en `poses.yaml`.
4. **Agarre:** cierra el gripper para sujetar la figura.
5. **Transporte seguro:**
   - retorna a `HOME2` antes de ir a la caneca para asegurar altura.
6. **Deposición:** llega a la coordenada de la caneca asignada (ej: `caneca_roja`) y abre el gripper.
7. **Retorno:** regresa a `HOME2` siguiendo la ruta inversa segura.

## 3. Comandos de Operación
Para verificar el funcionamiento de cada rutina sin necesidad de la cámara, se deben ejecutar los siguientes comandos en una terminal con el entorno ROS2 cargado:

Clasificar CUBO (Caneca Roja):

ros2 topic pub /figure_type std_msgs/msg/String "data: 'cubo'" --once
Clasificar CILINDRO (Caneca Verde):

ros2 topic pub /figure_type std_msgs/msg/String "data: 'cilindro'" --once
Clasificar PENTÁGONO (Caneca Azul):

ros2 topic pub /figure_type std_msgs/msg/String "data: 'pentagono'" --once
Clasificar RECTÁNGULO (Caneca Amarilla):

ros2 topic pub /figure_type std_msgs/msg/String "data: 'rectangulo'" --once


### Rutina de clasificacion de nodo
 Diagrama tipo rqt_graph o equivalente, explicando tópicos/servicios/acciones y flujo de datos.

### Mastil y canastilla 
### Video
#### simulacion
#### robot real

## Parte 2
### xacro actualizado
### Nodo de control por teclado
 Diagrama tipo rqt_graph o equivalente, explicando tópicos/servicios/acciones y flujo de datos.
### Nodo de control de ventosa
 Diagrama tipo rqt_graph o equivalente, explicando tópicos/servicios/acciones y flujo de datos.
### Foto/diagrama del circuito y explicación de cómo se controla desde el sistema.
### Lista de paquetes creados, propósito, nodos principales y cómo ejecutarlos.




