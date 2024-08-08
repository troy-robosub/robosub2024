from motion import arm, disarm, movement_control
from dvl import send_dvl_command
import json
import socket
import argparse
import rospy
import subprocess

#dependencies for alternate movement

#pymavlink to communicate with pixhawk
from pymavlink import mavutil
#timing
import time
#math operations
import math


def pmove(distance): #distance in meters
  arm.arm_submarine()
  # dvl assisted moving
  DVL_IP = '192.168.137.101'  # Update this with your DVL's IP address
  DVL_PORT = 16171
  # Connect to the DVL
  dvl_socket = send_dvl_command.connect_to_dvl(DVL_IP, DVL_PORT)
  if not dvl_socket:
      print("Failed to connect to DVL")
      return
  
  send_dvl_command.send_command(dvl_socket, "calibrate_gyro")
  send_dvl_command.send_command(dvl_socket, "reset_dead_reckoning")
  #subprocess.Popen(['python3', 'dvl/dvl_tcp_parser.py', 'dead_reckoning', '-i', '192.168.137.101'])
  #subprocess.Popen(['python3', 'dvl/dvl_publisher.py'])

  # if rospy doesn't work try using dvl_tcp_parser.py directly
  while True:
    response_json = send_dvl_command.send_command(dvl_socket, "dead_reckoning")
    print(response_json)
    print("i got x value: %s" % str(response_json["x"]))
    time.sleep(1)

  disarm.disarm_submarine()

pmove(1)

  
