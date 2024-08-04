#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, SetMode

def arm_submarine():
    rospy.init_node('arm_submarine_node', anonymous=True)
    
    # Wait for the arming and mode services to be available
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    
    try:
        # Create service proxies
        arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Set the flight mode to MANUAL
        mode_response = set_mode_service(custom_mode='MANUAL')
        if mode_response.mode_sent:
            rospy.loginfo("Mode set to MANUAL")
        else:
            rospy.logwarn("Failed to set mode")
        
        # Arm the vehicle
        arm_response = arming_service(True)
        if arm_response.success:
            rospy.loginfo("Submarine armed successfully")
        else:
            rospy.logwarn("Failed to arm the submarine")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        arm_submarine()
    except rospy.ROSInterruptException:
        pass
