#!/usr/bin/env python3

import rospy
# assume mavros_msgs package is installed already
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

def arm_submarine():
    rospy.init_node('arm_submarine_node', anonymous=True)

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        arm_response = arming_service(True)
        if arm_response.success:
            rospy.loginfo("Submarine armed successfully")
        else:
            rospy.logwarn("Failed to arm the submarine")
    except rospy.ServiceException as e:
        rospy.logerr(f"Arming failed: {e}")

if __name__ == '__main__':
    try:
        arm_submarine()
    except rospy.ROSInterruptException:
        pass

