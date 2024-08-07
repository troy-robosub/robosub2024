t#!/usr/bin/env python 

# Krishna and Kaileo, with help of Tim

# Make sure roscore is running before executing this script!

# This script uses rospy.publish(). 

import std_msgs
from std_msgs.msg import String
import cv2 
from ultralytics import YOLO
import numpy as np
import rospy 

model = YOLO("best50.pt")
cap = cv2.VideoCapture(-1)
classes = {0: 'bin', 1: 'bin_bleue', 2: 'bin_rouge', 3: 'buoy', 4: 'gate', 5: 'gate_bleue', 6: 'gate_rouge', 7: 'path', 8: 'torpille', 9: 'torpille_target'}
frame_size = (1920, 1080)
horizontal_degrees_view = 80 # 80 degrees for the lowlight cam, change when diff camera
frame_center = (frame_size[0] / 2, frame_size[1] / 2)

def find_angle_to_object(x1,y1,x2,y2):
    # W KRISHNA WHO FOUND TIM'S CODE 
    object_center = ((x1 + x2) // 2), ((y1 + y2 )// 2)
    
    hori_angle = (x2-x1)/1920 * horizontal_degrees_view
    
    return(hori_angle)

def inference(): 
    pub = rospy.Publisher('/cam/front', String, queue_size=10) #try changing the 'chatter' part
    rospy.init_node('inference', anonymous = True)
    rate = rospy.Rate(10)
    
    while (True):
        success, frame = cap.read()

        if success:
            # run inference on one frame
            results = model(source=frame, conf=0.3, show = True)

            # get data from inference in ultralytics.engine.results.boxes object
            # contains ordered lists with detection data
            boxes = results[0].boxes
        
            out = str(len(boxes.cls))

            # for each detected object in the current frame
            for i in range(len(boxes.cls)):
            # data will be outputted through ros topic
                left = float(boxes.xyxy[i][0])
                top = float(boxes.xyxy[i][1])
                right = float(boxes.xyxy[i][2])
                bottom = float(boxes.xyxy[i][3])
                
                out += "{" + classes[int(boxes.cls[i])] + " " + str(left) + " " + str(top) + " " + str(right) + " " + str(bottom) + "} "
                
                rospy.loginfo(out)
                pub.publish(out)
                rate.sleep()
                
            # format
            # numberofobjects {object1class object1top object1left object1bottom object1right object1anglefromcenter} {object2class object2top object2left object2bottom object2right object1anglefromcenter}


        # shows the stuff
        #cv2.imshow("YOLOv8 Inference (live)", results[0].plot())
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        else:
            break

while (True): 
    if __name__ == '__main__':
        try:
            inference()
        except rospy.ROSInterruptException:
            pass
