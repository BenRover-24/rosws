import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/celeste/workspace/benrover/rosws/install/benrover_sensors'
