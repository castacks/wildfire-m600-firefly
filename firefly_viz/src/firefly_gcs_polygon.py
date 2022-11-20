import rospy
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
import math
import rospkg
import yaml
origin = None
def clockwiseangle(point):
    global origin
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi
    x_axis = [0.0,1.0]
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*x_axis[0] + normalized[1]*x_axis[1]     # x1*x2 + y1*y2
    diffprod = x_axis[1]*normalized[0] - x_axis[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle
    return angle

def load_poly_from_file(empty_msg):
    global origin
    rospack = rospkg.RosPack()
    file = open(rospack.get_path("firefly_viz") + "/src/polygon.yaml")
    pt_dict = yaml.safe_load(file)
    pts_list = []
    poly_viz_list = []
    display_polygon = PolygonStamped()
    display_polygon.header.frame_id = "world"
    display_polygon.header.stamp = rospy.Time.now()
    polygon = PolygonStamped()
    polygon.header.frame_id = "world"
    polygon.header.stamp = rospy.Time.now()
    polygon.header.seq = 0
    for pt in pt_dict['outer_polygon']['points']:
        curr_pt = [pt['x'], pt['y']]
        pts_list.append(curr_pt)
    origin = list(np.mean(pts_list, axis=0))
    pts_list = sorted(pts_list, key= clockwiseangle)
    poly_viz_list += pts_list
    poly_viz_list.append(poly_viz_list[0])
    for pt in pts_list:
        point = Point32()
        point.x = pt[0]
        point.y = pt[1]
        point.z = 0.0
        polygon.polygon.points.append(point)
    rospy.loginfo("publishing poly outer")
    polygon_pub.publish(polygon)
    try:
        for hole_id, hole in enumerate(pt_dict['holes']):
            pts_list = []
            for pt in hole['points']:
                curr_pt = [pt['x'], pt['y']]
                pts_list.append(curr_pt)
            origin = np.mean(pts_list, axis=0).flatten()
            poly_viz_list += sorted(pts_list, key= clockwiseangle)
            poly_viz_list.append(sorted(pts_list, key= clockwiseangle)[0])
            pts_list = sorted(pts_list, key= clockwiseangle, reverse=True)
            polygon.polygon.points = []
            polygon.header.seq = hole_id + 1
            polygon.header.stamp = rospy.Time.now()
            for pt in pts_list:
                point = Point32()
                point.x = pt[0]
                point.y = pt[1]
                point.z = hole_id + 1
                polygon.polygon.points.append(point)
            rospy.loginfo("publishing poly hole")
            polygon_pub.publish(polygon)
    except:
        pass
    for pt in poly_viz_list:
        point = Point32()
        point.x = pt[0]
        point.y = pt[1]
        point.z = 0.0
        display_polygon.polygon.points.append(point)
    polygon_rviz.publish(display_polygon)                                   

    file.close()

if __name__ == '__main__':
    rospy.init_node('gcs_polygon_node')
    polygon_pub = rospy.Publisher('coverage_poly', PolygonStamped, queue_size=1)
    polygon_rviz = rospy.Publisher('display_poly', PolygonStamped, queue_size=1)
    load_polygon = rospy.Subscriber("view_coverage_poly", Empty, load_poly_from_file)
    rospy.spin()
