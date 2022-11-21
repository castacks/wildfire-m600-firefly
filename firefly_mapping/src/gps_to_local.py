#!/usr/bin/env python3

import rospy
import pymap3d as pm
import yaml
import rospkg
from nav_msgs.msg import OccupancyGrid

if __name__ == "__main__":
    rospy.init_node("gps_to_local", anonymous=True)
    pub = rospy.Publisher("gt_map", OccupancyGrid, queue_size=100)
    rospy.sleep(1)

    rospack = rospkg.RosPack()
    with open(rospack.get_path("firefly_mapping") + "/config/gps_hotspots.yaml", 'r') as f:
        d = yaml.load(f, Loader=yaml.FullLoader)

    while not rospy.has_param("local_pos_ref"):
        continue
    lat0, lon0, h0 = rospy.get_param("local_pos_ref")

    all_keys = list(d.keys())
    xys = []
    for i in range(0, len(all_keys), 3):
        x, y, _ = pm.enu.geodetic2enu(d[all_keys[i]], d[all_keys[i + 1]], d[all_keys[i + 2]], lat0, lon0, h0)
        xys.extend([x.item(), y.item()])

    rospy.set_param("gt_locs", xys)

    while not (rospy.has_param("~min_x") and rospy.has_param("~max_x") and rospy.has_param("~min_y") and rospy.has_param("~max_y") and rospy.has_param("~resolution")):
        continue

    resolution = rospy.get_param("~resolution")
    minX = rospy.get_param("~min_x")
    maxX = rospy.get_param("~max_x")
    minY = rospy.get_param("~min_y")
    maxY = rospy.get_param("~max_y")

    map_width = int((maxX - minX) / resolution)
    map_height = int((maxY - minY) / resolution)

    output = OccupancyGrid()
    output.header.frame_id = "world"
    output.info.resolution = resolution
    output.info.width = map_width
    output.info.height = map_height
    output.info.origin.position.x = minX
    output.info.origin.position.y = minY
    output.data = [0] * (map_width * map_height)

    for i in range(len(xys)//2):
        x = xys[2*i]
        y = xys[2*i+1]

        gtRow = int((y-minY)/resolution)
        gtCol = int((x-minX)/resolution)

        gtBin = gtCol + gtRow * map_width

        output.data[gtBin] = 100

    pub.publish(output)
    rospy.sleep(1)

    rospy.signal_shutdown("Set ground truth fire locations")