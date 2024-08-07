from motion import arm_submarine, disarm_submarine, movement_control
from dvl import send_dvl_command
import json
import rospy
import subprocess

def main():
  arm_submarine.arm_submarine
  # dvl assisted moving
  send_dvl_command.main("calibrate_gyro")
  send_dvl_command.main("reset_dead_reckoning")
  subprocess.Popen('roscore', shell=True)
  #subprocess.Popen(['python3', 'dvl/dvl_tcp_parser.py', 'dead_reckoning', '-i', '192.168.137.101'])
  subprocess.Popen(['python3', 'dvl/dvl_publisher.py'])

# if rospy doesn't work try using dvl_tcp_parser.py directly
  while not rospy.is_shutdown():
    with open('dvl/out.json', 'r') as f:
    	data = json.load(f)
    if data["x"] > 9.5:
    	break
    movement_control.movement(forward=1)

  disarm_submarine.disarm_submarine()

main()
  
