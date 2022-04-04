import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil
import sys
import time

connection = mavutil.mavlink_connection('/dev/mavlink', baud=57600)

# rate = rospy.Rate(rospy.get_param('~hz', 1))

def callback(data):
    #string_alt = "Height : " + str(data.altitude)
    #string_lat = "Lat : " + str(data.latitude)
    #string_lon = "Lon : " + str(data.longitude)


    
    #string = string_alt + string_lat + string_lon
    # rospy.loginfo("DATA PUB MAVLINK : ")
    #print(string)

    #connection.mav.named_value_float_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,"hei".encode(), data.altitude)
    #connection.mav.safety_allowed_area_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,  data.latitude, data.longitude, data.altitude, 0, 0, 0)
    connection.mav.safety_allowed_area_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,  data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, 0, 0, 0)
    # print("test")



def listener():
    rospy.init_node("mavlink_pub")
    print("Node init ")
    #rospy.Subscriber("/dji_sdk/gps_position",  NavSatFix , callback)
    rospy.Subscriber("/dji_sdk/imu",  Imu, callback)
    print("Subscribe init ")
    rospy.spin()

if __name__ == "__main__":
    print("Time to publish ")
    listener()
