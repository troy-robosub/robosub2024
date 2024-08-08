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
  #arm.arm_submarine()
  # dvl assisted moving
  send_dvl_command.main("calibrate_gyro")
  send_dvl_command.main("reset_dead_reckoning")
  #subprocess.Popen(['python3', 'dvl/dvl_tcp_parser.py', 'dead_reckoning', '-i', '192.168.137.101'])
  #subprocess.Popen(['python3', 'dvl/dvl_publisher.py'])

  # if rospy doesn't work try using dvl_tcp_parser.py directly
  listener()

  #disarm.disarm_submarine()

def callback(data):
    #convert data to a JSON entry
    json_data = json.loads(data.data)
    if float(json_data["x"]) > .238:
    	print("stop moving forward")
    	#code to stop moving forward
    else:
    	print("moving forward")
    	#code to start moving forward

def listener():
    # Initialize the ROS node
    rospy.init_node('exlistener', anonymous=True)
    
    # Subscribe to the topic
    rospy.Subscriber("information", String, callback)
    
    # Keep the script running
    rospy.spin()
pmove(1)

  
