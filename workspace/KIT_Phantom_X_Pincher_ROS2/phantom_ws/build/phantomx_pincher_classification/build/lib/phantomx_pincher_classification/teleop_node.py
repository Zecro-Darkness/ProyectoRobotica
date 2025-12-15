#!/usr/bin/env python3
"""
Nodo de teleoperación por teclado para PhantomX Pincher.
Controles:
- W/S: X +/-
- A/D: Y +/-
- Q/E: Z +/-
- R/F: Gripper Abrir/Cerrar
- ESPACIO: Reset a Home
"""

import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from phantomx_pincher_interfaces.msg import PoseCommand
from example_interfaces.msg import Bool

# Configuración
STEP_SIZE = 0.005  # 5 mm por paso (muy fino para control preciso)
HOME_POS = {'x': 0.128, 'y': 0.0, 'z': 0.150, 'roll': 3.142, 'pitch': 0.0, 'yaw': 0.0}  # Home original (abajo)
HOME2_POS = {'x': 0.100, 'y': 0.0, 'z': 0.200, 'roll': 0.0, 'pitch': -1.57, 'yaw': 0.0}  # Home vertical (arriba)

# Límites del espacio de trabajo (basados en poses.yaml conocidas)
WORKSPACE_LIMITS = {
    'x': (-0.05, 0.20),   # De -5cm a 20cm
    'y': (-0.15, 0.15),   # ±15cm
    'z': (0.04, 0.20)     # De 4cm a 20cm (evitar colisión con mesa)
}

msg = """
---------------------------
Control por Teclado
---------------------------
Moverse:
   Q    W    E
   A    S    D

W/S: X (Adelante/Atrás)
A/D: Y (Izquierda/Derecha)
Q/E: Z (Arriba/Abajo)

Gripper:
R: Abrir
F: Cerrar

Posiciones:
ESPACIO: Home (abajo)
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
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        self.pose_pub = self.create_publisher(PoseCommand, '/pose_command', 10)
        self.gripper_pub = self.create_publisher(Bool, '/open_gripper', 10)
        
        # Estado inicial (Home)
        self.x = HOME_POS['x']
        self.y = HOME_POS['y']
        self.z = HOME_POS['z']
        self.roll = HOME_POS['roll']
        self.pitch = HOME_POS['pitch']
        self.yaw = HOME_POS['yaw']

        self.get_logger().info('Nodo Teleop iniciado.')

    def clamp_position(self):
        """Limita las coordenadas al espacio de trabajo alcanzable."""
        self.x = max(WORKSPACE_LIMITS['x'][0], min(self.x, WORKSPACE_LIMITS['x'][1]))
        self.y = max(WORKSPACE_LIMITS['y'][0], min(self.y, WORKSPACE_LIMITS['y'][1]))
        self.z = max(WORKSPACE_LIMITS['z'][0], min(self.z, WORKSPACE_LIMITS['z'][1]))

    def publish_pose(self):
        msg = PoseCommand()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        msg.roll = float(self.roll)
        msg.pitch = float(self.pitch)
        msg.yaw = float(self.yaw)
        # Usar cartesian_path=True para movimientos más fluidos/lineales entre pasos cortos
        msg.cartesian_path = False
        
        self.pose_pub.publish(msg)
        print(f"\rEnviando: x={self.x:.3f} y={self.y:.3f} z={self.z:.3f}   ", end='')
        sys.stdout.flush()

    def control_gripper(self, open_grip):
        msg = Bool()
        msg.data = open_grip
        self.gripper_pub.publish(msg)
        print("\rGripper: " + ("Abierto" if open_grip else "Cerrado"), end='')

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopNode()
    
    print(msg)
    
    # Esperar a que el robot inicialice (importante para hardware real)
    print("Esperando inicialización del robot (3 segundos)...")
    time.sleep(3.0)
    
    # Enviar a Home al iniciar
    print("Enviando a Home inicial...")
    node.publish_pose()

    try:
        while True:
            key = getKey()
            if key == '\x03': # CTRL-C
                break
            
            updated = False
            
            if key == 'w':
                node.x += STEP_SIZE
                node.clamp_position()
                updated = True
            elif key == 's':
                node.x -= STEP_SIZE
                node.clamp_position()
                updated = True
            elif key == 'a':
                node.y += STEP_SIZE * 2  # Paso más grande para Y (lateral)
                node.clamp_position()
                updated = True
            elif key == 'd':
                node.y -= STEP_SIZE * 2  # Paso más grande para Y (lateral)
                node.clamp_position()
                updated = True
            elif key == 'q':
                node.z += STEP_SIZE
                node.clamp_position()
                updated = True
            elif key == 'e':
                node.z -= STEP_SIZE
                node.clamp_position()
                updated = True
            elif key == ' ': # SPACE - Home original
                node.x = HOME_POS['x']
                node.y = HOME_POS['y']
                node.z = HOME_POS['z']
                node.roll = HOME_POS['roll']
                node.pitch = HOME_POS['pitch']
                node.yaw = HOME_POS['yaw']
                print("\rHome (abajo)   ", end='')
                sys.stdout.flush()
                updated = True
            elif key == 'h': # H - Home2 vertical
                node.x = HOME2_POS['x']
                node.y = HOME2_POS['y']
                node.z = HOME2_POS['z']
                node.roll = HOME2_POS['roll']
                node.pitch = HOME2_POS['pitch']
                node.yaw = HOME2_POS['yaw']
                print("\rHome2 (vertical)   ", end='')
                sys.stdout.flush()
                updated = True
            elif key == 'r':
                node.control_gripper(True)
            elif key == 'f':
                node.control_gripper(False)

            if updated:
                node.publish_pose()

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
