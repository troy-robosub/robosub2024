from motion import arm_submarine, disarm_submarine, movement_control
from dvl import send_dvl_command
import json

def main():
  arm_submarine.arm_submarine()
  send_dvl_command.main("calibrate_gyro")
  send_dvl_command.main("reset_dead_reckoning")
  subprocess.Popen('roscore', shell=True)
  subprocess.Popen(['python3', '/dvl/dvl_tcp_parser.py', 'dead_reckoning', '-i', '192.168.137.101'])
  #subprocess.Popen(['python3', '/dvl/dvl_publisher.py'])
  
  while True:
    #movement code
    
    with open('/dvl/out.json','r') as f:
			data = json.load(f)
    if data["x"] >10:
      break
  disarm_submarine.disarm_submarine()
  
