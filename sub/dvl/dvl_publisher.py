#! /usr/bin/env python3
import rospy
import os
import json
from std_msgs.msg import String
import subprocess
def dvl_parse():
	subprocess.Popen(['python3', 'dvl_tcp_parser.py', 'dead_reckoning', '-i', '192.168.137.101'])
	topicName='information'
	#publishing to topic
	publisher1=rospy.Publisher(topicName,String,queue_size=5)
	
	#1Hz means 1 message per second
	ratePublisher=rospy.Rate(1)
	
	while not rospy.is_shutdown():
		try:
			with open('out.json','r') as f:
				data = json.load(f)
				data_str=json.dumps(data)
		except (json.JSONDecodeError):
			data_str="JSON Decode Error!"
		#rospy.loginfo(data_str)
		#publishing the message
		publisher1.publish(data_str)
		
		ratePublisher.sleep()