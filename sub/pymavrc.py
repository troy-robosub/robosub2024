#pymavlink to communicate with pixhawk
from pymavlink import mavutil
#timing
import time
#math operations
import math

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

def status_loop(duration, delay=0.05):
        ''' Loop for 'duration', with 'delay' between iterations. [s]
        Useful for debugging.
        '''
        start = time.time()
        print("running status_loop")
        while time.time() - start < duration:
            time.sleep(delay)

def clear_motion(self, stopped_pwm=1500):
        ''' Sets all 6 motion direction RC inputs to 'stopped_pwm'. '''
        print("running clear_motion")
        send_rc(*[stopped_pwm]*6)

# master = mavutil.mavlink_connection('192.168.2.2', baud=57600) #establish connection with pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600) #establish connection with pixhawk 
target = (master.target_system, master.target_component)

print("<<<<<<WAITING FOR CONNECTION>>>>>>")
print("Hi")
master.wait_heartbeat() #ensure connection is valid
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")

# for arming
master.arducopter_arm()

set_mode("STABILIZE")

send_rc("throttle=1700")

status_loop(2.0)

send_rc("throttle=1700")
status_loop(4.0)

clear_motion()
master.arducopter_disarm()


# for i in range(0,40):
#     print(getDepth(1007))
#     time.sleep(0.5)

# time.sleep(40)