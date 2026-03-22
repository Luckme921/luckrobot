import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nvidia/luckrobot/luckrobot_ws/install/luckrobot_launch1'
