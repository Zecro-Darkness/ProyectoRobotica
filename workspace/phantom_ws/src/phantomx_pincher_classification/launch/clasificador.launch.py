from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file para el sistema de clasificación del PhantomX Pincher.
    
    Lanza:
    - clasificador_node: Nodo que ejecuta las secuencias de pick-and-place
    """
    
    return LaunchDescription([
        # Nodo de clasificación
        Node(
            package='phantomx_pincher_classification',
            executable='clasificador_node',
            name='clasificador_node',
            output='screen',
            parameters=[],
            remappings=[],
        ),
    ])
