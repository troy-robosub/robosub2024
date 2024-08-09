import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
from std_msgs.msg import Float64, Float32MultiArray
import camera_ros.py


## using mavros to apply major motion

rospy.init_node('move')
pub_thrusters = rospy.Publisher("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn, queue_size=10)
# horizontal degrees view is the point where we make the anlge off of. 
horizontal_degrees_view = 
'''def find_angle_to_object(x1,y1,x2,y2):
    # W KRISHNA WHO FOUND TIM'S CODE 
    object_center = ((x1 + x2) // 2), ((y1 + y2 )// 2)
    hori_angle = (x2-x1)/1920 * horizontal_degrees_view
    amtx = 500 - object_center
    return(hori_angle)
newangle = find_angle_to_object(left,top,right,bottom)
dont need y values for this. 
'''
def obj_distance (x1,x2):
    target = (x1+x2)/2
    return (target)
x = obj_distance (left,right)
target_difference = 500 - |x|
def adjust_to_target (objdist, targetdiff): 
    while (targetdiff > 20):
        pwm = mavros_msgs.msg.OverrideRCIn()
        channels = [1500] * 18
        if objdist > 500: 
            # need to go right
            #make the left motors have higher power than right.  
            channels[4] = int (1900)
        else: 
        # need to go left, elif can be used too. 
        # make the right motors have higher power than left. 
            channels[4] = int (1100)
    
    # publishes the pwm to make the motors move. 
    #in the conditional loop change the yaw (channel 4). 1000-1100 is left, 2000-1900 is right. 
    pwm.channels = channels
    print(f"[INFO] Channels sent to pixhawk = {pwm}")
    pub_thrusters.publish(pwm)
adjust_to_target(x,target_difference)



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





"""
    def send_rc(self, rcin1=65535, rcin2=65535, rcin3=65535, rcin4=65535,
                rcin5=65535, rcin6=65535, rcin7=65535, rcin8=65535,
                rcin9=65535, rcin10=65535, rcin11=65535, rcin12=65535,
                rcin13=65535, rcin14=65535, rcin15=65535, rcin16=65535,
                rcin17=65535, rcin18=65535, *, # keyword-only from here
                pitch=None, roll=None, throttle=None, yaw=None, forward=None,
                lateral=None, camera_pan=None, camera_tilt=None, lights1=None,
                lights2=None, video_switch=None):
        ''' Sets all 18 rc channels as specified.
        Values should be between 1100-1900, or left as 65535 to ignore.
        Can specify values:
            positionally,
            or with rcinX (X=1-18),
            or with default RC Input channel mapping names
              -> see https://ardusub.com/developers/rc-input-and-output.html
        It's possible to mix and match specifier types (although generally
          not recommended). Default channel mapping names override positional
          or rcinX specifiers.
        '''
        rc_channel_values = (
            pitch        or rcin1,
            roll         or rcin2,
            throttle     or rcin3,
            yaw          or rcin4,
            forward      or rcin5,
            lateral      or rcin6,
            camera_pan   or rcin7,
            camera_tilt  or rcin8,
            lights1      or rcin9,
            lights2      or rcin10,
            video_switch or rcin11,
            rcin12, rcin13, rcin14, rcin15, rcin16, rcin17, rcin18
        )
        logging.info(f'send_rc')
        logging.debug(rc_channel_values)
        self.mav.rc_channels_override_send(
            *self.target,
            *rc_channel_values
        )
"""




# code for issue number 4: angle turning task. 
# desciprtion: use the data collected in sub/vision/camera_ros.py and utilize the angle return function. 
# Then, write code to minimize the angle of that through turning. Write this code for special turning within 
# movement_control.py. For the turning command, just put a placeholder for now.