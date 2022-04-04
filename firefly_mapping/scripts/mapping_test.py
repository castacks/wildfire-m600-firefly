#!/usr/bin/env python

import rospy
from firefly_mapping.msg import ImageWithPose
from sensor_msgs.msg import Image
import json
from PIL import Image as PILImage
from PIL import ImageOps
import rospkg
import numpy as np

if __name__ == '__main__':
    rospy.init_node('projection_tester', anonymous=True)
    pub = rospy.Publisher('image_to_project', ImageWithPose, queue_size=10)
    pub1 = rospy.Publisher('image', Image, queue_size=10)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('firefly_mapping')

    im_path = package_path + "/test_data/test_imgs/" + rospy.get_param('~image', "square50.png")
    pose_path = package_path + "/test_data/test_poses/" + rospy.get_param('~pose', "down_at_30m.json")

    msg = ImageWithPose()

    with open(pose_path) as json_file:
        data = json.load(json_file)
        msg.pose.position.x = data['x']
        msg.pose.position.y = data['y']
        msg.pose.position.z = data['z']
        msg.pose.orientation.x = data['qx']
        msg.pose.orientation.y = data['qy']
        msg.pose.orientation.z = data['qz']
        msg.pose.orientation.w = data['qw']

    with PILImage.open(im_path) as im:
        msg.image.width = im.width
        msg.image.height = im.height
        msg.image.data = list(np.array(ImageOps.grayscale(im)).reshape(-1))
        msg.image.encoding = 'mono8'


    rospy.sleep(1.0)
    pub.publish(msg)
    pub1.publish(msg.image)
