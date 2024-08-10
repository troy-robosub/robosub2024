#pymavlink to communicate with pixhawk
from pymavlink import mavutil
#timing
import time
# for multi-tasking
import threading
#math operations
import math
#dvl stuff
from dvl.dvl_publisher import dvl_parse
from dvl import send_dvl_command
import signal
import sys
import json
import rospy
from std_msgs.msg import String


#global for dvl

current_x=0.0
current_z=0.0

def set_mode(modep):
    mode = modep
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system, #target system
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, #custom_mode is the mode we are setting
        mode_id) #mode_id is the mode we are setting it to (previous code)
    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")

def send_rc(rcin1=65535, rcin2=65535, rcin3=65535, rcin4=65535,
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
        print("running send_rc")
        print("rc_channel_values", rc_channel_values)

        master.mav.rc_channels_override_send(
            *target,
            *rc_channel_values
        )

#check for heading in Qgroundcontrol
def get_heading():
    heading = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                heading = data[3].split(",")[0]
            except:
                print('')

            heading = float(heading)
            break

    return heading

def forward(duration, pwm=1700): #default pwm is 1600, but can definitely adjust, and duration is in seconds
    send_rc(forward=pwm)
    for i in range(duration * 20):
        send_rc()
        time.sleep(0.05)

    clear_motion()

def down(duration, pwm=1400): #default pwm is 1600, but can definitely adjust, and duration is in seconds
    send_rc(throttle=pwm)
    for i in range(duration * 20):
        send_rc()
        time.sleep(0.05)

    clear_motion()

def down(duration, pwm=1600): #default pwm is 1600, but can definitely adjust, and duration is in seconds
    send_rc(throttle=pwm)
    for i in range(duration * 20):
        send_rc()
        time.sleep(0.05)

    clear_motion()

# def turnHeading(heading):
#     #hold altitude
#     mode = 'ALT_HOLD'
#     mode_id = master.mode_mapping()[mode]
#     master.mav.set_mode_send(
#         master.target_system,
#         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#         mode_id)
#     #get the current heading
#     start_heading = get_heading()
#     #calculate the difference in headings
#     angle = abs(start_heading - heading) #derek you put abs bruh it cant be negative we changed it to float were lowkey goated
#     #if the angle is negative, rotate clockwise (increase heading)
#     if (start_heading > heading):
#         while (start_heading > heading) {

def rotateClockwise(degrees):
    #hold altitude and send message
    print("start heading: " + str(get_heading()))

    #the original heading of the vehicle.
    #what is heading? the compass direction in which the craft's nose is pointing
    #https://mavlink.io/en/messages/common.html#ATTITUDE
    start_heading = get_heading()
    new_heading = (start_heading + degrees) % 360
    #run this 10000000 times
    for i in range(10000000):
        print(i) # does it only run this loop once?
        #get current heading, usually after small shift in yaw
        current_heading = get_heading()
        #calculate the difference in rotation by degrees
        #rotate clockwise, no thrust
        if current_heading > 0.75 * new_heading and current_heading < 1.25 * new_heading:
            send_rc(yaw=1525)
        else:
            send_rc(yaw=1550)
        print("current heading: " + str(get_heading()))
        # if the desired degrees rotated
        # is greater than the desired rotation (within 4%), stop
        if current_heading > 0.90 * new_heading and  current_heading < 1.1 * new_heading:
            break
    #print the rotation reached
        print("ROTATED: ", current_heading)
    #hold altitude

    clear_motion()

def rotateCounterClockwise(degrees):
    set_mode("MANUAL")
    print("start heading: " + str(get_heading()))

    #the original heading of the vehicle.
    #what is heading? the compass direction in which the craft's nose is pointing
    #https://mavlink.io/en/messages/common.html#ATTITUDE
    start_heading = get_heading()
    new_heading = (start_heading - degrees) % 360
    #run this 10000000 times
    for i in range(10000000):
        print(i) # does it only run this loop once?
        #get current heading, usually after small shift in yaw
        current_heading = get_heading()
        #calculate the difference in rotation by degrees
        #rotate clockwise, no thrust
        if current_heading > 0.75 * new_heading and current_heading < 1.25 * new_heading:
            send_rc(yaw=1475)
        else:
            send_rc(yaw=1450)
        print("current heading: " + str(get_heading()))
        # if the desired degrees rotated
        # is greater than the desired rotation (within 4%), stop
        if current_heading > 0.90 * new_heading and current_heading < 1.00 * new_heading:
            break
    #print the rotation reached
        print("ROTATED: ", current_heading)
    #hold altitude

    clear_motion()
    
def maintainHeading(heading):
    set_mode("ALT_HOLD")
    #get the current heading
    start_heading = get_heading()
    #calculate the difference in headings
    angle = float(start_heading - heading) #derek you put abs bruh it cant be negative we changed it to float were lowkey goated
    #if the angle is negative, rotate clockwise (increase heading)
    if angle < 0: 
        rotateClockwise(abs(angle))
    #if the angle is positive, rotate counter-clockwise (decrease heading)
    else: 
        rotateCounterClockwise(abs(angle))
def status_loop(duration, delay=0.05):
        ''' Loop for 'duration', with 'delay' between iterations. [s]
        Useful for debugging.
        '''
        start = time.time()
        print("running status_loop")
        while time.time() - start < duration:
            time.sleep(delay)

def clear_motion(stopped_pwm=1500):
        ''' Sets all 6 motion direction RC inputs to 'stopped_pwm'. '''
        print("running clear_motion")
        send_rc(*[stopped_pwm]*6)

def dvl_callback(data):
    global current_x
    global current_z
    try:
        data_dict = json.loads(data.data)
        current_x = data_dict.get("x", 0.0)
        current_z = data_dict.get("z", 0.0)
        print(current_x)
        print(current_z)
    except json.JSONDecodeError:
        rospy.logerr("JSON Decode Error!")

def dvl_forward(distance,pwm=1700):
     send_dvl_command.main('reset_dead_reckoning')
     send_rc(forward=pwm)
     while current_x <= distance:
        print(current_x)
        # Replace this with your actual movement command
        send_rc()
        time.sleep(0.05)  # Small delay to allow for callback updates

def dvl_down(depth, pwm=1400):
    send_dvl_command.main('reset_dead_reckoning')
    send_rc(throttle=pwm)
    while current_z <= depth:
        print(current_z)
        send_rc()
        time.sleep(0.05)

def signal_handler(signal, frame):
    set_mode("MANUAL")
    clear_motion()
    print("Ctrl+C pressed, exiting...")
    sys.exit(0)

def subscriber_thread():
    rospy.Subscriber('information', String, dvl_callback)
    rospy.spin()

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# master = mavutil.mavlink_connection('192.168.2.2', baud=57600) #establish connection with pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600) #establish connection with pixhawk
target = (master.target_system, master.target_component)

print("<<<<<<WAITING FOR CONNECTION>>>>>>")
print("Hi")
master.wait_heartbeat() #ensure connection is valid
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")

clear_motion()

#send_dvl_command.main('calibrate_gyro')
#send_dvl_command.main('reset_dead_reckoning')

#begin parsing dead reckoning
print("STARTING DEAD RECKONING...")
time.sleep(2)
#rospy.init_node('main_node', anonymous=True)
#dvl_publisher.dvl_parse()

#thread = threading.Thread(target=dvl_parse)
#thread.start()

#thread2 = threading.Thread(target=subscriber_thread)
#thread2.start()

# for arming
master.arducopter_arm()
set_mode("STABILIZE")

down(1)
forward(2)
#dvl_forward(2)
set_mode("MANUAL")
clear_motion()
master.arducopter_disarm()


# for i in range(0,40):
#     print(getDepth(1007))
#     time.sleep(0.5)

# time.sleep(40)
#pymavlink to communicate with pixhawk
