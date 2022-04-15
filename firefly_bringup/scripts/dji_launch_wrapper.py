import os
import signal
import rospy
from sensor_msgs.msg import BatteryState
import roslaunch
from threading import Lock

pro = None
last_msg_time = rospy.Time(0)
last_msg_time_lock = Lock()
launched_dji = False
proc = None


def battery_callback(msg):
    global last_msg_time
    with last_msg_time_lock:
        last_msg_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('dji_launch_wrapper', anonymous=True)
    rospy.Subscriber("dji_sdk/battery_state", BatteryState, battery_callback)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_file = roslaunch.rlutil.resolve_launch_arguments(['dji_sdk', 'sdk.launch'])

    while not rospy.is_shutdown():
        if launched_dji:
            with last_msg_time_lock:
                temp = last_msg_time
            if (rospy.Time.now() - temp).to_sec() > 1.0:
                print("Shutting down dji_sdk sdk.launch")
                launch.shutdown()
                launched_dji = False
                rospy.sleep(10)
        else:
            if os.path.exists("/dev/dji"):
                print("Launching dji_sdk sdk.launch")
                launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file, force_screen=True)
                launch.start()
                launched_dji = True
                rospy.sleep(15)

        rospy.sleep(0.05)