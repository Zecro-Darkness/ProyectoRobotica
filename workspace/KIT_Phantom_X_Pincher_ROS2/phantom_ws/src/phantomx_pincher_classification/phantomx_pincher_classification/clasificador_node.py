#!/usr/bin/env python3
"""
Nodo de clasificación para PhantomX Pincher.

Este nodo orquesta operaciones de pick-and-place basadas en el tipo de figura detectada.
Suscribe al tipo de figura y publica comandos de pose para mover el robot a través de
una secuencia completa de recolección y colocación en la caneca correcta.

Mapeo de figuras a canecas:
- cubo → caneca_roja
- cilindro → caneca_verde
- pentagono → caneca_azul
- rectangulo → caneca_amarilla
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from phantomx_pincher_interfaces.msg import PoseCommand
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from enum import Enum


class SequenceState(Enum):
    """Estados de la secuencia de pick-and-place"""
    IDLE = 0
    MOVING_TO_HOME_START = 1
    OPENING_GRIPPER_START = 2
    MOVING_TO_PICKUP = 3
    CLOSING_GRIPPER = 4
    MOVING_TO_HOME_WITH_OBJECT = 5
    MOVING_TO_SAFE_POS_1 = 6
    MOVING_TO_SAFE_POS_2 = 12
    MOVING_TO_SAFE_POS_3 = 14
    MOVING_TO_SAFE_POS_4 = 16
    MOVING_TO_BIN = 7
    OPENING_GRIPPER_DROP = 8
    RETURNING_TO_SAFE_POS_4 = 17
    RETURNING_TO_SAFE_POS_3 = 15
    RETURNING_TO_SAFE_POS_2 = 13
    RETURNING_TO_SAFE_POS_1 = 9
    RETURNING_TO_HOME_END = 10
    COMPLETED = 11


class ClasificadorNode(Node):
    """
    Nodo que ejecuta secuencias de pick-and-place basadas en el tipo de figura.
    """

    def __init__(self):
        super().__init__('clasificador_node')

        # Mapeo de figura a caneca
        self.figure_to_bin = {
            'cubo': 'caneca_roja',
            'cilindro': 'caneca_verde',
            'pentagono': 'caneca_azul',
            'rectangulo': 'caneca_amarilla'
        }

        # --- CONFIGURACIÓN DE TIEMPOS ---
        self.TIME_MOVEMENT = 2  # Tiempo para movimientos del brazo
        self.TIME_GRIPPER = 1   # Tiempo para abrir/cerrar gripper
        # -------------------------------

        # Estado de la secuencia
        self.current_state = SequenceState.IDLE
        self.current_bin = None
        self.current_figure = None

        # Cargar poses desde el archivo YAML
        self.poses = self.load_poses()
        self.get_logger().info(f'Poses cargadas: {list(self.poses.keys())}')

        # Publisher para comandos de pose
        self.pose_pub = self.create_publisher(
            PoseCommand,
            '/pose_command',
            10
        )

        # Action client para control del gripper
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            'gripper_trajectory_controller/follow_joint_trajectory'
        )
        
        # Nombres de los joints del gripper
        self.gripper_joint_names = [
            'phantomx_pincher_gripper_finger1_joint',
            'phantomx_pincher_gripper_finger2_joint'
        ]

        # Subscriber para tipo de figura
        self.figure_sub = self.create_subscription(
            String,
            '/figure_type',
            self.figure_callback,
            10
        )

        # Timer para ejecutar la secuencia paso a paso
        self.sequence_timer = None

        self.get_logger().info('Nodo clasificador iniciado y listo para recibir comandos')
        self.get_logger().info(f'Mapeo de figuras: {self.figure_to_bin}')

    def load_poses(self):
        """
        Carga las poses desde el archivo poses.yaml del paquete phantomx_pincher_bringup.
        
        Returns:
            dict: Diccionario con las poses cargadas
        """
        try:
            # Obtener ruta al paquete de configuración
            bringup_share = get_package_share_directory('phantomx_pincher_bringup')
            poses_path = os.path.join(bringup_share, 'config', 'poses.yaml')
            
            self.get_logger().info(f'Cargando poses desde: {poses_path}')
            
            with open(poses_path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('poses', {})
                
        except Exception as e:
            self.get_logger().error(f'Error cargando poses: {e}')
            # Retornar poses por defecto en caso de error
            return {
                'home': {'x': 0.15, 'y': 0.0, 'z': 0.2, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
                'recoleccion': {'x': 0.1, 'y': 0.0, 'z': 0.038, 'roll': 3.142, 'pitch': -0.007, 'yaw': 0.0}
            }

    def publish_pose(self, pose_name, cartesian_path=False):
        """
        Publica un comando de pose al tópico /pose_command.
        
        Args:
            pose_name (str): Nombre de la pose en el diccionario
            cartesian_path (bool): Si usar trayectoria cartesiana
        """
        if pose_name not in self.poses:
            self.get_logger().error(f'Pose "{pose_name}" no encontrada en configuración')
            return False

        pose = self.poses[pose_name]
        
        msg = PoseCommand()
        msg.x = float(pose['x'])
        msg.y = float(pose['y'])
        msg.z = float(pose['z'])
        msg.roll = float(pose['roll'])
        msg.pitch = float(pose['pitch'])
        msg.yaw = float(pose['yaw'])
        msg.cartesian_path = cartesian_path

        self.get_logger().info(
            f'📍 Publicando pose "{pose_name}": '
            f'x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, '
            f'roll={msg.roll:.3f}, pitch={msg.pitch:.3f}, yaw={msg.yaw:.3f}, '
            f'cartesian={msg.cartesian_path}'
        )

        self.pose_pub.publish(msg)
        return True

    def control_gripper(self, open_gripper):
        """
        Controla el gripper (abrir o cerrar) usando el action client.
        
        Args:
            open_gripper (bool): True para abrir, False para cerrar
        """
        action = "🔓 Abriendo" if open_gripper else "🔒 Cerrando"
        self.get_logger().info(f'{action} gripper...')
        
        # Crear goal para el gripper
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joint_names
        
        point = JointTrajectoryPoint()
        # Posiciones: 1.0 para abrir (~57°), 0.5 para cerrar (~29°)
        position = 1.4 if open_gripper else 0.5
        point.positions = [position, position]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        goal.trajectory.points = [point]
        
        # Enviar goal de forma asíncrona (no bloqueante)
        self.gripper_client.send_goal_async(goal)

    def execute_sequence_step(self):
        """
        Ejecuta el siguiente paso de la secuencia.
        Llamado por el timer.
        """
        if self.current_state == SequenceState.IDLE:
            return

        # 1. Ir a HOME
        elif self.current_state == SequenceState.MOVING_TO_HOME_START:
            self.get_logger().info('🏠 [Paso 1/12] Ir a HOME...')
            if self.publish_pose('home', cartesian_path=False):
                # Skip Gripper -> Ir directo a Recolección
                self.current_state = SequenceState.OPENING_GRIPPER_START
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a HOME')
                self.abort_sequence()

        # 1.5 Abrir Gripper (Inicio)
        elif self.current_state == SequenceState.OPENING_GRIPPER_START:
            self.control_gripper(True)
            self.current_state = SequenceState.MOVING_TO_PICKUP
            self.schedule_next_step(self.TIME_GRIPPER)

        # 2. Zona Recolección
        elif self.current_state == SequenceState.MOVING_TO_PICKUP:
            self.get_logger().info('📦 [Paso 2/12] Ir a RECOLECCIÓN...')
            if self.publish_pose('recoleccion', cartesian_path=False):
                 self.current_state = SequenceState.CLOSING_GRIPPER
                 self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a RECOLECCIÓN')
                self.abort_sequence()

        # 2.5 Cerrar Gripper
        elif self.current_state == SequenceState.CLOSING_GRIPPER:
            self.control_gripper(False)
            
            # Si es CUBO, RECTANGULO, PENTAGONO o CILINDRO, ir a HOME antes de las posiciones seguras para rotar
            if self.current_figure in ['cubo', 'rectangulo', 'pentagono', 'cilindro']:
                self.current_state = SequenceState.MOVING_TO_HOME_WITH_OBJECT
            else:
                # Para otras figuras, comportamiento original: Skip Home-Lift -> Ir directo a SAFE_CARRY_1
                self.current_state = SequenceState.MOVING_TO_SAFE_POS_1
            
            self.schedule_next_step(self.TIME_GRIPPER)

        # 3. [SKIPPED] Ir a HOME (Levantar) -> REPLACED BY SAFE_CARRY_1 LOGIC
        
        # 3. Ir a HOME con objeto (Solo Cubo)
        elif self.current_state == SequenceState.MOVING_TO_HOME_WITH_OBJECT:
            self.get_logger().info('🏠 [Paso Extra] Ir a HOME con objeto...')
            if self.publish_pose('home', cartesian_path=False):
                if self.current_figure == 'cubo':
                    self.current_state = SequenceState.MOVING_TO_BIN
                elif self.current_figure == 'rectangulo':
                    # Directo a Cañeca (desde Home)
                    self.current_state = SequenceState.MOVING_TO_BIN
                elif self.current_figure == 'pentagono':
                    self.current_state = SequenceState.MOVING_TO_BIN
                elif self.current_figure == 'cilindro':
                    self.current_state = SequenceState.MOVING_TO_BIN
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a HOME con objeto')
                self.abort_sequence()





        # 4. Ir a Posición Segura 1
        elif self.current_state == SequenceState.MOVING_TO_SAFE_POS_1:
            self.get_logger().info('⬆️  [Paso 3/12] Ir a SAFE_CARRY_1...')
            if self.publish_pose('safe_carry_1', cartesian_path=False):
                self.current_state = SequenceState.MOVING_TO_SAFE_POS_2
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a safe_carry_1')
                self.abort_sequence()

        # 4. Ir a Posición Segura 2
        elif self.current_state == SequenceState.MOVING_TO_SAFE_POS_2:
            self.get_logger().info('↗️  [Paso 4/12] Ir a SAFE_CARRY_2...')
            if self.publish_pose('safe_carry_2', cartesian_path=False):
                self.current_state = SequenceState.MOVING_TO_SAFE_POS_3
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a safe_carry_2')
                self.abort_sequence()

        # 5. Ir a Posición Segura 3
        elif self.current_state == SequenceState.MOVING_TO_SAFE_POS_3:
            self.get_logger().info('↗️  [Paso 5/12] Ir a SAFE_CARRY_3...')
            if self.publish_pose('safe_carry_3', cartesian_path=False):
                self.current_state = SequenceState.MOVING_TO_SAFE_POS_4
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a safe_carry_3')
                self.abort_sequence()

        # 6. Ir a Posición Segura 4
        elif self.current_state == SequenceState.MOVING_TO_SAFE_POS_4:
            self.get_logger().info('↗️  [Paso 6/12] Ir a SAFE_CARRY_4...')
            if self.publish_pose('safe_carry_4', cartesian_path=False):
                self.current_state = SequenceState.MOVING_TO_BIN
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo mover a safe_carry_4')
                self.abort_sequence()

        # 7. Ir a Caneca
        elif self.current_state == SequenceState.MOVING_TO_BIN:
            self.get_logger().info(f'🎯 [Paso 7/12] Ir a {self.current_bin.upper()}...')
            if self.publish_pose(self.current_bin, cartesian_path=False):
                self.current_state = SequenceState.OPENING_GRIPPER_DROP
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error(f'❌ Error: No se pudo mover a {self.current_bin}')
                self.abort_sequence()

        # 7.5 Abrir Gripper (Soltar)
        elif self.current_state == SequenceState.OPENING_GRIPPER_DROP:
            self.control_gripper(True)

            # Retorno diferenciado para Cubo
            if self.current_figure == 'cubo':
                self.current_state = SequenceState.RETURNING_TO_HOME_END
            elif self.current_figure == 'rectangulo':
                # Retorno directo a HOME final
                self.current_state = SequenceState.RETURNING_TO_HOME_END
            elif self.current_figure == 'pentagono':
                self.current_state = SequenceState.RETURNING_TO_HOME_END
            elif self.current_figure == 'cilindro':
                self.current_state = SequenceState.RETURNING_TO_HOME_END
            else:
                self.current_state = SequenceState.RETURNING_TO_SAFE_POS_4
            
            self.schedule_next_step(self.TIME_GRIPPER)



        # 8. Ir a Posición Segura 4 (Retorno)
        elif self.current_state == SequenceState.RETURNING_TO_SAFE_POS_4:
            self.get_logger().info('↗️  [Paso 8/12] Ir a SAFE_CARRY_4 (Retorno)...')
            if self.publish_pose('safe_carry_4', cartesian_path=False):
                self.current_state = SequenceState.RETURNING_TO_SAFE_POS_3
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo regresar a safe_carry_4')
                self.abort_sequence()

        # 9. Ir a Posición Segura 3 (Retorno)
        elif self.current_state == SequenceState.RETURNING_TO_SAFE_POS_3:
            self.get_logger().info('↗️  [Paso 9/12] Ir a SAFE_CARRY_3 (Retorno)...')
            if self.publish_pose('safe_carry_3', cartesian_path=False):
                self.current_state = SequenceState.RETURNING_TO_SAFE_POS_2
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo regresar a safe_carry_3')
                self.abort_sequence()

        # 10. Ir a Posición Segura 2 (Retorno)
        elif self.current_state == SequenceState.RETURNING_TO_SAFE_POS_2:
            self.get_logger().info('↗️  [Paso 10/12] Ir a SAFE_CARRY_2 (Retorno)...')
            if self.publish_pose('safe_carry_2', cartesian_path=False):
                self.current_state = SequenceState.RETURNING_TO_SAFE_POS_1
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo regresar a safe_carry_2')
                self.abort_sequence()

        # 11. Ir a Posición Segura 1 (Retorno)
        elif self.current_state == SequenceState.RETURNING_TO_SAFE_POS_1:
            self.get_logger().info('⬆️  [Paso 11/12] Ir a SAFE_CARRY_1 (Retorno)...')
            if self.publish_pose('safe_carry_1', cartesian_path=False):
                self.current_state = SequenceState.RETURNING_TO_HOME_END
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo regresar a safe_carry_1')
                self.abort_sequence()

        # 12. Ir a HOME (Final)
        elif self.current_state == SequenceState.RETURNING_TO_HOME_END:
            self.get_logger().info('🏠 [Paso 12/12] Ir a HOME (Final)...')
            if self.publish_pose('home', cartesian_path=False):
                self.current_state = SequenceState.COMPLETED
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error('❌ Error: No se pudo regresar a HOME final')
                self.abort_sequence()

        elif self.current_state == SequenceState.COMPLETED:
            self.get_logger().info('=' * 60)
            self.get_logger().info('✅ SECUENCIA COMPLETADA EXITOSAMENTE')
            self.get_logger().info('=' * 60)
            self.current_state = SequenceState.IDLE
            if self.sequence_timer:
                self.sequence_timer.cancel()
                self.sequence_timer = None

    def schedule_next_step(self, delay_seconds):
        """
        Programa el siguiente paso de la secuencia.
        
        Args:
            delay_seconds (float): Tiempo de espera antes del siguiente paso
        """
        if self.sequence_timer:
            self.sequence_timer.cancel()
        
        self.sequence_timer = self.create_timer(
            delay_seconds,
            self.execute_sequence_step
        )

    def abort_sequence(self):
        """
        Aborta la secuencia actual.
        """
        self.get_logger().error('❌ Secuencia abortada debido a un error')
        self.current_state = SequenceState.IDLE
        if self.sequence_timer:
            self.sequence_timer.cancel()
            self.sequence_timer = None

    def start_sequence(self, figure_type):
        """
        Inicia una nueva secuencia de pick-and-place.
        
        Args:
            figure_type (str): Tipo de figura (cubo, cilindro, pentagono, rectangulo)
        """
        # Validar tipo de figura
        if figure_type not in self.figure_to_bin:
            self.get_logger().error(
                f'Tipo de figura "{figure_type}" no reconocido. '
                f'Tipos válidos: {list(self.figure_to_bin.keys())}'
            )
            return

        # Verificar que no haya una secuencia en curso
        if self.current_state != SequenceState.IDLE:
            self.get_logger().warn('⚠️  Ya hay una secuencia en curso. Ignorando comando.')
            return

        self.current_bin = self.figure_to_bin[figure_type]
        self.current_figure = figure_type
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🚀 INICIANDO SECUENCIA ROBUSTA DE 10 PASOS')
        self.get_logger().info(f'📋 Figura: {figure_type} → Caneca: {self.current_bin}')
        self.get_logger().info('=' * 60)

        # Iniciar la secuencia en el paso 1
        self.current_state = SequenceState.MOVING_TO_HOME_START
        self.execute_sequence_step()

    def figure_callback(self, msg):
        """
        Callback para el tópico /figure_type.
        Ejecuta la secuencia de pick-and-place cuando se recibe un tipo de figura.
        
        Args:
            msg (String): Mensaje con el tipo de figura
        """
        figure_type = msg.data.lower().strip()
        
        self.get_logger().info(f'📨 Recibido tipo de figura: "{figure_type}"')
        
        # Iniciar secuencia de pick-and-place
        self.start_sequence(figure_type)


def main(args=None):
    """
    Punto de entrada del nodo.
    """
    rclpy.init(args=args)
    
    try:
        node = ClasificadorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error en clasificador_node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
