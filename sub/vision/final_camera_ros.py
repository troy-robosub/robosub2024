#!/usr/bin/env python 

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

#import std_msgs
#from std_msgs.msg import String
import cv2 
import math #as meth
from ultralytics import YOLO
import numpy as np
#import rospy
import cvzone
import torch

line = 1
cap = cv2.VideoCapture("GX010017.mp4")

#device = torch.device('cpu')
cap.set(3,720)
cap.set(4,1280)
totx, toty = 640, 485
centerx, centery = int(totx/2), int(toty/2)



model = YOLO("front_cam_comp_data.pt")
#device=torch.device("cpu")

#classes = {0: 'counterclockwise Banner', 1: 'clockwise Banner', 2: 'Buoy', 3: 'Torpedo Banner', 4: 'Torpedo Holes', 5: 'Full Gate'} #arvp
classes = {0: 'gateblue', 1: 'gatered', 2: 'buoy', 3: 'torpedoes', 4: 'torpedo hole', 5: 'gate', 6: 'torpille', 7: 'y', 8: 'x', 9: ' samples_table', 10: ' path'}
frame_size = (totx, toty)
frame_center = (frame_size[0] / 2, frame_size[1] / 2)
confidence = 0.8

def inference(Obsticle): 
    #pub = rospy.Publisher('/cam/front', String, queue_size=10) #try changing the 'chatter' part
    #rospy.init_node('inference', anonymous = True)
    #rate = rospy.Rate(10)

    while True:
        success, img = cap.read()
        results = model(img, stream = True, conf=confidence)

        img = cv2.line(img, (centerx,0), (centerx,toty), (0,255,0), line)
        img = cv2.line(img, (0,centery), (totx,centery), (0,255,0), line)
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
            
                if classes[cls] == "gate"  and conf>confidence and Obsticle == "gate":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(0,0,255),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("gate")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)

                if classes[cls] == "bin" and conf > confidence and Obsticle == "bin":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("bin")
                    return(getdirection(centerx, centery, c_x, c_y ), x1, x2)

                if classes[cls] == "buoy" and conf > confidence and Obsticle == "buoy":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("buoy")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)

                if classes[cls] == "gatered" and conf > confidence and Obsticle == "gatered":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("gatered")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)

                if classes[cls] == "gateblue" and conf > confidence and Obsticle == "gateblue":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("gateblue")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)

                if classes[cls] == "torpedo hole" and conf > confidence and Obsticle == "torpedo hole":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("torpedo hole")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)

                if classes[cls] == "binred" and conf > confidence and Obsticle == "binred":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("bin_rouge")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)

                if classes[cls] == "samples_table" and conf > confidence and Obsticle == "samples_table":
                    cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,0),3)
                    cvzone.putTextRect(img, f'{classes[cls]} {conf}',(max(0,x1),max(35,y1)), scale = 1, thickness=2)#defult is 3 for both
                    c_x = x1 + int((x2-x1)/2)
                    c_y = y1 + int((y2-y1)/2)
                    cv2.circle(img, (c_x,c_y), radius=10, color=(0, 255, 0), thickness=10)
                    print("samples_table")
                    a = [getdirection(centerx, centery, c_x, c_y), x1, x2]
                    return(a)
                
                # numberofobjects {object1class object1confidence object1top object1left object1bottom object1right}

            #for i in range(len(boxes.cls)):
                # data will be outputted through ros topic
            #    conf = float(boxes.confidence[i])
            #    left = float(boxes.xyxy[i][0])
            #    top = float(boxes.xyxy[i][1])
            #    right = float(boxes.xyxy[i][2])
            #    bottom = float(boxes.xyxy[i][3])

            # format
            # numberofobjects {object1class object1confidence  movetoCenter() object1top object1left object1bottom object1right}
            #out += " {" + classes[int(boxes.cls[i])] + " " + str(movetoCenter(centerx, centery, x1 + int((x2-x1)/2), y1 + int((y2-y1)/2))) + " " + str(conf) + " " + str(left) + " " + str(top) + " " + str(right) + " " + str(bottom) + "} "
            

        #pub.publish(out)

        cv2.imshow("Image", img)
        cv2.waitKey(1)

focallen=546.823529
def distance(truesize, x1, x2):
    return(truesize*focallen/abs(x1-x2)) #returns distance away based on units of truesize (currently inches but we can change it to wtv we want)

def getdirection(centerx, centery, currx, curry):
    threshx=300
    if currx-centerx>threshx:
        return "right"
    if currx-centerx<threshx*-1:
        return "left"
    else:
        return "maintain"

def gatered(direction, x1, x2):
    while direction == "right":
        print("move right")  # strafe right code
        a = inference("gatered")
        direction = a[0]  # Update direction
        if direction != "right":  # Exit loop if direction is not "right"
            break

    while direction == "left":
        print("move left")  # strafe left code
        a = inference("gatered")
        direction = a[0]  # Update direction
        if direction != "left":  # Exit loop if direction is not "left"
            break

    if direction == "maintain":
        # moves forward code for like 10 ft or about
        print("move gate red distance:", int(distance(12, x1, x2)))
        return


def buoy(direction, x1, x2):
    while direction == "right":
        print("move right")  # strafe right code
        a = inference("buoy")
        direction = a[0]  # Update direction
        if direction != "right":  # Exit loop if direction is not "right"
            break

    while direction == "left":
        print("move left")  # strafe left code
        a = inference("buoy")
        direction = a[0]  # Update direction
        if direction != "left":  # Exit loop if direction is not "left"
            break

    if direction == "maintain":
        # moves forward code for like 10 ft or about
        print("move buoy distance:", int(distance(9, x1, x2)))
        return




def strategy():
    #look for gate
    a = inference("gatered")
    gatered(a[0], a[1], a[2])
    a = inference("buoy")
    buoy(a[0], a[1], a[2])

strategy()

#if __name__ == '__main__':
#    try:
#        inference()
#    except False: #rospy.ROSInterruptException:
#        pass
