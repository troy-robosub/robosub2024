import rospy
from mavros_msgs.srv import CommandBool, SetMode
from arm import arm_submarine
from disarm import disarm_submarine
## using mavros to apply major motion
arm_submarine()
disarm_submarine()

# https://www.ardusub.com/developers/rc-input-and-output.html

# https://github.com/InspirationRobotics/robosub_2024/blob/dac720839070f41935d3b328c7454b22f64cfa0c/auv/motion/robot_control.py#L2
