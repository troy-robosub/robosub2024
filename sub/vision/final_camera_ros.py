#!/usr/bin/env python 

# Krishna and Kaileo, with help of Tim

# Make sure roscore is running before executing this script!

# This script does not use rospy.publish(). 

#import std_msgs
#from std_msgs.msg import String
import cv2 
import math
from ultralytics import YOLO
import numpy as np
#import rospy
import cvzone

import std_msgs
from std_msgs.msg import String
import cv2 
import math
from ultralytics import YOLO
import numpy as np
import rospy
import cvzone
import torch

line = 100
cap = cv2.VideoCapture(0)

#device = torch.device('cpu')
cap.set(3,1920)
cap.set(4,1080)
totx, toty = 1920, 1080
centerx, centery = int(totx/2), int(toty/2)

def movetoCenter(centerx, centery, currx, curry):
    threshx=100
    if currx-centerx>threshx:
        return "left"
    if currx-centerx<threshx:
        return "right"
    else:
        return "maintain"

model = YOLO("front_cam_comp_data.pt")
device=torch.device("cpu")
cap = cv2.VideoCapture(0)

classes = {0: 'counterclockwise Banner', 1: 'clockwise Banner', 2: 'Buoy', 3: 'Torpedo Banner', 4: 'Torpedo Holes', 5: 'Full Gate'}
frame_size = (totx, toty)
frame_center = (frame_size[0] / 2, frame_size[1] / 2)
confidence = 0.5

def inference(): 
    pub = rospy.Publisher('/cam/front', String, queue_size=10) #try changing the 'chatter' part
    rospy.init_node('inference', anonymous = True)
    rate = rospy.Rate(10)

    while True:
        success, img = cap.read()
        results = model(img, stream = True, conf=confidence)

        cv2.line(img, (centerx,0), (centerx,toty), (0,255,0), line)
        cv2.line(img, (0,centery), (totx,centery), (0,255,0), line)
        cv2.imshow("image", img)


        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2  = int(x1), int(y1), int(x2), int(y2)
                print(x1, y1, x2, y2)
                object_center = ((x1 + x2) // 2), ((y1 + y2 )// 2)
                #cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)

                conf = math.ceil((box.conf[0]*100))/100
                cls = int(box.cls[0])
            
                if classes[cls] == "Full Gate"  and conf>confidence:
                    cv2.rectangle(img,(x1,y1),(x2,y2),(0,0,255),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("gate")
                    print(movetoCenter(centerx, centery, c_x, c_y ))

                if classes[cls] == "Counterclockwise Banner" and conf > confidence:
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("gate_blue")
                    print(movetoCenter(centerx, centery, c_x, c_y ))

                if classes[cls] == "Buoy" and conf > confidence:
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("buoy")
                    print(movetoCenter(centerx, centery, c_x, c_y ))
                
                # numberofobjects {object1class object1confidence object1top object1left object1bottom object1right}
                for i in range(len(boxes.cls)):
            # data will be outputted through ros topic
                    conf = float(boxes.confidence[i])
                    left = float(boxes.xyxy[i][0])
                    top = float(boxes.xyxy[i][1])
                    right = float(boxes.xyxy[i][2])
                    bottom = float(boxes.xyxy[i][3])
            
            # format
            # numberofobjects {object1class object1confidence object1top object1left object1bottom object1right}
                    out += " {" + classes[int(boxes.cls[i])] + " " + str(conf) + " " + str(left) + " " + str(top) + " " + str(right) + " " + str(bottom) + "} "
            

        pub.publish(out)

        cv2.imshow("Image", img)
        cv2.waitKey(1)



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
