#!/usr/bin/env python 

# Krishna and Kaileo, with help of Tim

# Make sure roscore is running before executing this script!

# This script does not use rospy.publish(). 

import std_msgs
from std_msgs.msg import String
import cv2 
from ultralytics import YOLO
import numpy as np
import rospy 

model = YOLO("best50.pt")
cap = cv2.VideoCapture(-1)
classes = {0: 'bin', 1: 'bin_bleue', 2: 'bin_rouge', 3: 'buoy', 4: 'gate', 5: 'gate_bleue', 6: 'gate_rouge', 7: 'path', 8: 'torpille', 9: 'torpille_target'}
frame_size = (640, 480)
frame_center = (frame_size[0] / 2, frame_size[1] / 2)

def find_angle_to_object(x1,y1,x2,y2):
    # W KRISHNA WHO FOUND TIM'S CODE 
    frame_center = (320, 320)
    object_center = ((x1 + x2) // 2), ((y1 + y2 )// 2)
    
    angle = np.degrees(np.arctan2(object_center[1] - frame_center[1], 
                                  object_center[0] - frame_center[0]))
    return(angle)

def inference(): 
    pub = rospy.Publisher('chatter', String, queue_size=10) #try changing the 'chatter' part
    rospy.init_node('inference', anonymous = True)
    rate = rospy.Rate(10)
    
    while (True):
        success, frame = cap.read()

        if success:
            results = model(source=frame, conf=0.7, imgsz=640)

            boxes = results[0].boxes
            
            for i in range(len(boxes.cls)):
                # print statements will be changed to rospy publish stuff
                print("")
                print(classes[int(boxes.cls[i])])
                print("confidence:", float(boxes.conf[i]))
                left = float(boxes.xyxy[i][0])
                top = float(boxes.xyxy[i][1])
                right = float(boxes.xyxy[i][2])
                bottom = float(boxes.xyxy[i][3])

                print("left:", left)
                print("top:", top)
                print("right:", right)
                print("bottom:", bottom)
                print("centroid: (" + str((float(boxes.xyxy[i][0]) + float(boxes.xyxy[i][2])) * 0.5) + ", " + str((float(boxes.xyxy[i][1]) + float(boxes.xyxy[i][3])) * 0.5) + ")")
                print("offset from center: " + str(320 - (float(boxes.xyxy[i][0]) + float(boxes.xyxy[i][2])) * 0.5) + ", " + str(320 - (float(boxes.xyxy[i][1]) + float(boxes.xyxy[i][3])) * 0.5) + ")")
                print(find_angle_to_object(left, top, right, bottom))

            cv2.imshow("YOLOv8 Inference (live)", results[0].plot())

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            break

if __name__ == '__main__':
    try:
        inference()
    except rospy.ROSInterruptException:
        pass









"""



while (True):
    success, frame = cap.read()

    if success:
        results = model(source=frame, conf=0.7, imgsz=640)

        boxes = results[0].boxes
        for i in range(len(boxes.cls)):
            # print statements will be changed to rospy publish stuff
            print("")
            print(classes[int(boxes.cls[i])])
            print("confidence:", float(boxes.conf[i]))
            left = float(boxes.xyxy[i][0])
            top = float(boxes.xyxy[i][1])
            right = float(boxes.xyxy[i][2])
            bottom = float(boxes.xyxy[i][3])

            print("left:", left)
            print("top:", top)
            print("right:", right)
            print("bottom:", bottom)
            print("centroid: (" + str((float(boxes.xyxy[i][0]) + float(boxes.xyxy[i][2])) * 0.5) + ", " + str((float(boxes.xyxy[i][1]) + float(boxes.xyxy[i][3])) * 0.5) + ")")
            print("offset from center: " + str(320 - (float(boxes.xyxy[i][0]) + float(boxes.xyxy[i][2])) * 0.5) + ", " + str(320 - (float(boxes.xyxy[i][1]) + float(boxes.xyxy[i][3])) * 0.5) + ")")
            print(find_angle_to_object(left, top, right, bottom))

        cv2.imshow("YOLOv8 Inference (live)", results[0].plot())

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break
"""
