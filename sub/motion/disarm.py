#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, SetMode

def disarm_submarine():
    rospy.init_node('disarm_submarine_node', anonymous=True)
    
    # Wait for the arming and mode services to be available
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    
    try:
        # Create service proxies
        arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Set flight mode to stabilize
        mode_response = set_mode_service(custom_mode='STABILIZE')
        if mode_response.mode_sent:
            rospy.loginfo("Mode set to STABILIZE")
        else:
            rospy.logwarn("Failed to set mode")
        
        # Disarm the sub
        arm_response = arming_service(False)
        if arm_response.success:
            rospy.loginfo("Submarine disarmed successfully")
        else:
            rospy.logwarn("Failed to disarm the submarine")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
