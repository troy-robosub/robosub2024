import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
from std_msgs.msg import Float64, Float32MultiArray

## using mavros to apply major motion

rospy.init_node('move')
pub_thrusters = rospy.Publisher("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, queue_size=10)

def movement(
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=0,
        **kwargs,
    ):

        pwm = mavros_msgs.msg.OverrideRCIn()

        # Calculate PWM values
        channels = [1500] * 18
        # channels[2] = int((vertical * 80) + 1500) if vertical else 1500
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500
        channels[6] = int((pitch * 80) + 1500) if pitch else 1500
        channels[7] = int((roll * 80) + 1500) if roll else 1500
        pwm.channels = channels

        # Publish PWMs to /mavros/rc/override
        print(f"[INFO] Channels sent to pixhawk = {pwm}")
        pub_thrusters.publish(pwm)

#this uses pymavlink instead of mavros
# def movement(x, y, z, r):
#     master.mav.manual_control_send(
#         master.target_system,
#         x,  # pitch for sub, back/forward on joystick [-1000,1000], respectively
#         y,  # roll for sub, left/right on joystick [-1000,1000], respectively
#         z,  # thrust for sub, slider on joystick [0,1000]
#         r,  # yaw for sub, clockwise/counterclockwise on joystick [-1000,1000], respectively
#         0)  # buttons
#     time.sleep(0.05)

# https://www.ardusub.com/developers/rc-input-and-output.html

# https://github.com/InspirationRobotics/robosub_2024/blob/dac720839070f41935d3b328c7454b22f64cfa0c/auv/motion/robot_control.py#L2
