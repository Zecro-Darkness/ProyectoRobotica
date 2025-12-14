# Manipulación y Clasificación Robot PhantomX Pincher

## Integrantes

- Alejandro Mendivelso Torres
- Juan Manuel Beltran Botello 
- Oscar Jhondairo Siabato Leon
- Edgar Esteban Erazo Lagos


Conceptos Técnicos Clave Para entender y modificar este proyecto, es útil tener en cuenta los siguientes conceptos: 
- Máquina de Estados (State Machine)
El programa de clasificación no es una lista simple de instrucciones secuenciales (como una receta de cocina lineal), sino una Máquina de Estados. ¿Por qué? Porque en robótica las acciones tardan tiempo. Mover el brazo tarda 3 segundos. Abrir la pinza tarda 1 segundo. ¿Cómo funciona? El código sabe en qué "estado" está (ej: ABRIENDO_GRIPPER). Tiene un temporizador que chequea periódicamente. Cuando pasa el tiempo necesario, cambia el "estado" al siguiente paso lógico (ej: BAJANDO_A_MESA). Esto hace que el programa sea, robusto y no se congele.
- Cinemática Directa vs Inversa
En el Teleop, usamos lo que conceptualmente es cinemática directa: controlamos los ángulos ($q_1, q_2...$) y el robot llega a una posición resultante. Es muy seguro. En el Clasificador, usamos Cinemática Inversa (IK): Le decimos al robot "Ve a la coordenada X=20cm, Y=10cm". El robot debe calcular internamente cuánto girar cada motor para lograrlo. Si la coordenada está fuera del alcance físico del robot, el sistema reportará un error y abortará la secuencia para protegerse.
- Comunicación ROS2
El proyecto demuestra los dos pilares de comunicación en ROS: Topics (Tópicos): Usados para información rápida y unidireccional. El usuario o un sistema externo publica "cubo" en el tópico /figure_type, y el robot lo escucha para iniciar la tarea. Actions (Acciones): Usados para tareas largas. El nodo clasificador le dice al controlador "Muévete a esta posición". Como esto toma tiempo, usan un cliente de acción (FollowJointTrajectory) que permite esperar a que el movimiento termine antes de hacer otra cosa.

## 1. Resumen del Proyecto
Este proyecto implementa un sistema avanzado de control y automatización para el brazo robótico PhantomX Pincher utilizando ROS2 (Robot Operating System 2). El sistema está diseñado para realizar tareas de manipulación de objetos ("pick and place") de manera robusta y precisa.

El desarrollo abarca dos modos principales de operación:

Teleoperación Manual: Permite a un operador controlar cada articulación del robot, el efector final (gripper) y herramientas externas (bomba de vacío) utilizando el teclado del ordenador.
Clasificación Automática: Un sistema autónomo que recibe un comando específico indicando el tipo de objeto ("cubo", "cilindro", "pentágono", "rectángulo") y ejecuta una coreografía de movimientos precisa para recoger el objeto y depositarlo en su contenedor específico.

## 2. Descripción General del Proyecto
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
El objetivo de esta guía es crear una rutina de robot que pueda recoger 4 objetos distintos que siempre se ubicarán en la zona de recolección y luego depositarlos en la caneca correspondiente según su tipo.

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




