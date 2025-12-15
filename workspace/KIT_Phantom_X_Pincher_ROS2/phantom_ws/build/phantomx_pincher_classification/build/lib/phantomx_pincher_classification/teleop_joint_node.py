#!/usr/bin/env python3
"""
Nodo de teleoperación ARTICULAR por teclado para PhantomX Pincher.
Control directo de joints (más rápido y confiable que control cartesiano).

Controles:
- W/S: Joint 1 (Base - Rotación)
- A/D: Joint 2 (Hombro)
- Q/E: Joint 3 (Codo)
- Z/X: Joint 4 (Muñeca)
- R/F: Gripper Abrir/Cerrar
- ESPACIO: Home
- H: Home2 (vertical)
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import serial
import time

# Configuración
JOINT_STEP = 0.008  # 0.05 radianes por paso (~3 grados)

# Nombres de los joints del brazo
ARM_JOINT_NAMES = [
    'phantomx_pincher_arm_shoulder_pan_joint',    # Joint 1 - Base
    'phantomx_pincher_arm_shoulder_lift_joint',   # Joint 2 - Hombro  
    'phantomx_pincher_arm_elbow_flex_joint',      # Joint 3 - Codo
    'phantomx_pincher_arm_wrist_flex_joint'       # Joint 4 - Muñeca
]

GRIPPER_JOINT_NAMES = [
    'phantomx_pincher_gripper_finger1_joint',
    'phantomx_pincher_gripper_finger2_joint'
]

# Posiciones Home (radianes)
HOME_POSITIONS = [0.0, 0.0, 0.0, 0.0]  # Todos los joints en 0
HOME2_POSITIONS = [0.0, 0.0, 1.57, 1.57]  # Vertical hacia arriba

msg = """
---------------------------
Control Articular por Teclado
---------------------------
Joints:
W/S: Joint 2 (Hombro)
A/D: Joint 1 (Base)
Q/E: Joint 3 (Codo)
Z/X: Joint 4 (Muñeca)

Gripper:
R: Abrir
F: Cerrar

Bomba de Vacío (Relé):
O: Encender bomba
P: Apagar bomba

Posiciones:
ESPACIO: Home
H: Home2 (vertical)
CTRL-C: Salir
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSANOW, settings)
    return key

class TeleopJointNode(Node):
    def __init__(self):
        super().__init__('teleop_joint_node')
        
        # Action clients
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            'gripper_trajectory_controller/follow_joint_trajectory'
        )
        
        # Posiciones actuales de los joints
        self.joint_positions = list(HOME_POSITIONS)
        
        # Conexión Arduino para control de relé
        self.arduino = None
        
        # Declarar parámetro para puerto Arduino
        # Por defecto /dev/ttyACM0 (Arduino típico)
        # Robot usa /dev/ttyUSB1 (FTDI)
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.init_arduino()
        
        self.get_logger().info('Esperando action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('✓ Action servers conectados')
        
        # Suscriptor para leer estado actual (para inicio suave)
        self.current_joints_read = False
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
            
        self.get_logger().info('Nodo Teleop Articular iniciado.')

    def listener_callback(self, msg):
        """Actualiza la posición solo si no se ha inicializado."""
        if self.current_joints_read:
            return
            
        # Mapear posiciones del mensaje a nuestros joints
        try:
            current_pos = []
            for name in ARM_JOINT_NAMES:
                if name in msg.name:
                    idx = msg.name.index(name)
                    current_pos.append(msg.position[idx])
            
            if len(current_pos) == 4:
                self.joint_positions = current_pos
                self.current_joints_read = True
                self.get_logger().info('✓ Posición inicial sincronizada con el robot.')
        except Exception as e:
            pass

    def init_arduino(self):
        """Inicializa la conexión serial con Arduino."""
        arduino_port = self.get_parameter('arduino_port').value
        
        # Si no se especificó puerto, deshabilitar Arduino
        if not arduino_port:
            self.get_logger().info('ℹ Puerto Arduino no especificado. Control de relé deshabilitado.')
            self.get_logger().info('  Para habilitar: --ros-args -p arduino_port:=/dev/ttyACM0')
            self.arduino = None
            return
        
        try:
            self.arduino = serial.Serial(arduino_port, 9600, timeout=1)
            time.sleep(2)  # Esperar a que Arduino se reinicie
            self.get_logger().info(f'✓ Arduino conectado en {arduino_port}')
        except (serial.SerialException, FileNotFoundError) as e:
            self.get_logger().warn(f'⚠ No se pudo conectar Arduino en {arduino_port}')
            self.get_logger().info('  Control de relé deshabilitado. El robot funcionará normalmente.')
            self.arduino = None

    def send_relay_command(self, turn_on):
        """Envía comando al Arduino para controlar el relé."""
        if self.arduino is None:
            print("\r⚠ Arduino no conectado   ", end='')
            sys.stdout.flush()
            return
        
        try:
            command = 'O' if turn_on else 'P'
            self.arduino.write(command.encode())
            time.sleep(0.1)
            
            # Leer respuesta del Arduino
            if self.arduino.in_waiting > 0:
                response = self.arduino.readline().decode().strip()
                print(f"\r{response}   ", end='')
            else:
                estado = "ENCENDIDA" if turn_on else "APAGADA"
                print(f"\rBomba {estado}   ", end='')
            sys.stdout.flush()
        except Exception as e:
            self.get_logger().error(f'Error al enviar comando al Arduino: {e}')
            print(f"\r⚠ Error de comunicación con Arduino   ", end='')
            sys.stdout.flush()

    def send_arm_command(self, duration_sec=0.001):
        """Envía comando de posición al brazo.
        
        Args:
            duration_sec (float): Duración del movimiento en segundos (default: 0.5)
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal.trajectory.points = [point]
        
        self.arm_client.send_goal_async(goal)
        
        # Mostrar posiciones
        print(f"\rJoints: [{self.joint_positions[0]:.2f}, {self.joint_positions[1]:.2f}, "
              f"{self.joint_positions[2]:.2f}, {self.joint_positions[3]:.2f}]   ", end='')
        sys.stdout.flush()

    def send_gripper_command(self, open_gripper):
        """Envía comando al gripper."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINT_NAMES
        
        point = JointTrajectoryPoint()
        # Abierto: 0.5, Cerrado: -0.5 fr(en radianes, ~29 grados)
        position = 1.8 if open_gripper else 0.6
        point.positions = [position, position]
        point.time_from_start = Duration(sec=0, nanosec=200000000)
        
        goal.trajectory.points = [point]
        
        self.gripper_client.send_goal_async(goal)
        
        estado = "Abierto" if open_gripper else "Cerrado"
        print(f"\rGripper: {estado}   ", end='')
        sys.stdout.flush()

    def go_to_home(self, home2=False, slow=False):
        """Envía el robot a posición Home de forma progresiva."""
        target = HOME2_POSITIONS if home2 else HOME_POSITIONS
        
        print("\rYendo a Home..." + (" (Vertical)" if home2 else "") + "   ", end='')
        sys.stdout.flush()

        # Configuración de interpolación
        steps = 100  # Número de pasos intermedios
        delay = 0.05 # Tiempo entre pasos (segundos)
        
        # Calcular incrementos por paso
        deltas = []
        for i in range(4):
            delta = (target[i] - self.joint_positions[i]) / steps
            deltas.append(delta)
            
        # Ejecutar movimiento progresivo
        for _ in range(steps):
            for i in range(4):
                self.joint_positions[i] += deltas[i]
            
            # Enviar comando con duración corta para respuesta inmediata
            self.send_arm_command(duration_sec=delay)
            time.sleep(delay)
            
        # Asegurar posición final exacta
        self.joint_positions = list(target)
        self.send_arm_command(duration_sec=0.5)
        print("\rLlegada a Home completada.   ", end='')
        sys.stdout.flush()

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopJointNode()
    
    print(msg)
    
    # Sincronizar con el robot real antes de mover nada
    print("Sincronizando estado del robot (esperando /joint_states)...")
    while not node.current_joints_read:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Ir a Home al iniciar (movimiento suave desde la posición real)
    print("Enviando a Home inicial (interpolado)...")
    node.go_to_home(slow=True)

    try:
        while True:
            key = getKey()
            if key == '\x03': # CTRL-C
                break
            
            updated = False
            
            # Joint 2 - Hombro (W/S)
            if key == 'w':
                node.joint_positions[1] += JOINT_STEP
                updated = True
            elif key == 's':
                node.joint_positions[1] -= JOINT_STEP
                updated = True
            
            # Joint 1 - Base (A/D)
            elif key == 'a':
                node.joint_positions[0] += JOINT_STEP
                updated = True
            elif key == 'd':
                node.joint_positions[0] -= JOINT_STEP
                updated = True
            
            # Joint 3 - Codo (Q/E)
            elif key == 'q':
                node.joint_positions[2] += JOINT_STEP
                updated = True
            elif key == 'e':
                node.joint_positions[2] -= JOINT_STEP
                updated = True
            
            # Joint 4 - Muñeca (Z/X)
            elif key == 'z':
                node.joint_positions[3] += JOINT_STEP
                updated = True
            elif key == 'x':
                node.joint_positions[3] -= JOINT_STEP
                updated = True
            
            # Homes
            elif key == ' ': # SPACE
                node.go_to_home(home2=False)
            elif key == 'h':
                node.go_to_home(home2=True)
            
            # Gripper
            elif key == 'r':
                node.send_gripper_command(True)
            elif key == 'f':
                node.send_gripper_command(False)
            
            # Relé - Bomba de Vacío
            elif key == 'o' or key == 'O':
                node.send_relay_command(True)  # Encender bomba
            elif key == 'p' or key == 'P':
                node.send_relay_command(False)  # Apagar bomba

            if updated:
                node.send_arm_command()

    except Exception as e:
        print(e)
    finally:
        # Cerrar conexión Arduino si existe
        if node.arduino is not None:
            node.arduino.close()
            print("\nArduino desconectado.")
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
