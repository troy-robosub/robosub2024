from motion import arm_submarine, disarm_submarine
from dvl import send_dvl_command

def main():
  arm_submarine.arm_submarine()
  send_dvl_command.main("calibrate_gyro")
  send_dvl_command.main("reset_dead_reckoning")
  subprocess.Popen(['python3', '/dvl/dvl_tcp_parser.py', 'dead_reckoning', '-i', '192.168.137.101'])


  disarm_submarine.disarm_submarine()
  
