import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/phantom_ws/install/phantomx_pincher_classification'
