import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matan/ros2_ws/src/esp32_bridge_pkg/install/esp32_bridge_pkg'
