#!/usr/bin/env python3

import rospy
import pymap3d as pm
import yaml

class GPS2LocalENU:
    def __init__(self):
        with open('../config/gps_hotspots.yaml', 'r') as f:
            self.d = yaml.load(f, Loader=yaml.FullLoader)
        
        self.lat0, self.lon0, self.h0 = rospy.get_param("local_pos_ref")

    def gps2local(self):
        all_keys = list(self.d.keys())
        xys = []
        for i in range(0, len(all_keys), 3):
            x, y, _ = pm.enu.geodetic2enu(self.d[all_keys[i]], self.d[all_keys[i+1]], self.d[all_keys[i+2]], self.lat0, self.lon0, self.h0)
            xys.extend([x, y])
        
        rospy.set_param("gt_locs", xys)


if __name__ == "__main__":
    rospy.init_node("gps_to_local", anonymous=True)
    g2l = GPS2LocalENU()
    g2l.gps2local()

    # TODO: Check if param is still on server after shutting down node
    rospy.signal_shutdown("Set ground truth fire locations")