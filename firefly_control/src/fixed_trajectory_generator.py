#!/usr/bin/python3
import numpy as np
import rospy
from core_trajectory_msgs.msg import WaypointXYZVYaw
from core_trajectory_msgs.msg import TrajectoryXYZVYaw
from core_trajectory_msgs.msg import FixedTrajectory
import copy
from coverage_planner import get_polygon_path, Point2d, trapezoidal_decomposition, generate_cell_traversal, get_full_coverage_path
from firefly_telemetry.msg import PolygonArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

# global variables
polygonPointsList = []
current_odom = None

def get_velocities(traj, velocity, max_acc):
    v_prev = 0.

    for i in range(len(traj.waypoints)):
        j = (i+1) % len(traj.waypoints)
        dx = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy = traj.waypoints[j].position.y - traj.waypoints[i].position.y

        dist = np.sqrt(dx**2 + dy**2)
        v_limit = np.sqrt(v_prev**2 + 2*max_acc*dist)
        traj.waypoints[i].velocity = min(velocity, v_limit)
        v_prev = traj.waypoints[i].velocity



def get_racetrack_waypoints(attributes):#length, width, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    straightaway_length = length - width

    # first straightaway
    xs1 = np.linspace(0, straightaway_length, 20)
    ys1 = np.zeros(xs1.shape)
    yaws1 = np.zeros(xs1.shape)

    # first turn
    t = np.linspace(-np.pi/2, np.pi/2, 30)[1:-1]
    xs2 = width/2.*np.cos(t) + straightaway_length
    ys2 = width/2.*np.sin(t) + width/2.
    xs2d = -width/2.*np.sin(t) # derivative of xs
    ys2d = width/2.*np.cos(t) # derivative of ys
    yaws2 = np.arctan2(ys2d, xs2d)

    # second straightaway
    xs3 = np.linspace(straightaway_length, 0, 20)
    ys3 = width*np.ones(xs3.shape)
    yaws3 = np.pi*np.ones(xs3.shape)

    # second turn
    t = np.linspace(np.pi/2, 3*np.pi/2, 30)[1:-1]
    xs4 = width/2.*np.cos(t)
    ys4 = width/2.*np.sin(t) + width/2.
    yaws4 = yaws2 + np.pi

    xs = np.hstack((xs1, xs2, xs3, xs4))
    ys = np.hstack((ys1, ys2, ys3, ys4))
    yaws = np.hstack((yaws1, yaws2, yaws3, yaws4))

    now = rospy.Time.now()
    for i in range(xs.shape[0]):
        wp = WaypointXYZVYaw()
        wp.position.x = xs[i]
        wp.position.y = ys[i]
        wp.position.z = height
        wp.yaw = yaws[i]

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj


def get_figure8_waypoints(attributes):#length, width, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    now = rospy.Time.now()

    # figure 8 points
    t = np.linspace(0, 2*np.pi, 100)
    x = np.cos(t) * length - length
    y = np.cos(t)*np.sin(t) * 2*width

    # derivative of figure 8 curve, used to find yaw
    xd = -np.sin(t) * length
    yd = (np.cos(t)**2 - np.sin(t)**2) * 2*width

    for i in range(t.shape[0] - 1):
        x1 = x[i]
        y1 = y[i]
        x2 = x1 + xd[i]
        y2 = y1 + yd[i]

        yaw = np.arctan2(y2 - y1, x2 - x1)

        wp = WaypointXYZVYaw()
        wp.position.x = x1
        wp.position.y = y1
        wp.position.z = height
        wp.yaw = yaw

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_line_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for y in np.arange(0, -length, -0.5):
        wp = WaypointXYZVYaw()
        wp.position.x = 0
        wp.position.y = y
        wp.position.z = height
        wp.yaw = 0

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_point_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    x = float(attributes['x'])
    y = float(attributes['y'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    # add first point
    wp = WaypointXYZVYaw()
    wp.position.x = x
    wp.position.y = y
    wp.position.z = height
    wp.yaw = 0

    traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_box_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    wp1 = WaypointXYZVYaw()
    wp1.position.x = 0.
    wp1.position.y = 0.
    wp1.position.z = height
    wp1.yaw = 0
    traj.waypoints.append(wp1)

    wp2 = WaypointXYZVYaw()
    wp2.position.x = length
    wp2.position.y = 0.
    wp2.position.z = height
    wp2.yaw = 0
    traj.waypoints.append(wp2)

    wp3 = WaypointXYZVYaw()
    wp3.position.x = length
    wp3.position.y = 0.
    wp3.position.z = height + height
    wp3.yaw = 0
    traj.waypoints.append(wp3)

    wp4 = WaypointXYZVYaw()
    wp4.position.x = 0.
    wp4.position.y = 0.
    wp4.position.z = height + height
    wp4.yaw = 0
    traj.waypoints.append(wp4)

    return traj

def get_vertical_lawnmower_waypoints(attributes):#length, width, height, velocity):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for i in range(abs(int(height/width))):
        wp1 = WaypointXYZVYaw()
        wp1.position.x = 0
        wp1.position.y = 0
        wp1.position.z = np.sign(height)*(i+1)*width
        wp1.yaw = 0
        wp1.velocity = 0.1

        wp1_ = WaypointXYZVYaw()
        wp1_.position.x = 0
        wp1_.position.y = 0.5
        wp1_.position.z = np.sign(height)*(i+1)*width
        wp1_.yaw = 0
        wp1_.velocity = velocity

        wp2 = WaypointXYZVYaw()
        wp2.position.x = 0
        wp2.position.y = length
        wp2.position.z = np.sign(height)*(i+1)*width
        wp2.yaw = 0
        wp2.velocity = 0.1

        wp2_ = WaypointXYZVYaw()
        wp2_.position.x = 0
        wp2_.position.y = length - 0.5
        wp2_.position.z = np.sign(height)*(i+1)*width
        wp2_.yaw = 0
        wp2_.velocity = velocity

        if i%2 == 0:
            traj.waypoints.append(wp1)
            wp1_slow = copy.deepcopy(wp1_)
            wp1_slow.velocity = 0.1
            traj.waypoints.append(wp1_slow)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp2)
        else:
            traj.waypoints.append(wp2)
            wp2_slow = copy.deepcopy(wp2_)
            wp2_slow.velocity = 0.1
            traj.waypoints.append(wp2_slow)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp1)

    wp = WaypointXYZVYaw()
    wp.position.x = 0
    wp.position.y = 0
    wp.position.z = 0
    wp.yaw = 0
    wp.velocity = 0.5
    traj.waypoints.append(wp)

    return traj

def get_circle_waypoints(attributes):#radius, velocity, frame_id):
    frame_id = str(attributes['frame_id'])
    radius = float(attributes['radius'])
    velocity = float(attributes['velocity'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()

    wp0 = WaypointXYZVYaw()
    wp0.position.x = 0
    wp0.position.y = 0
    wp0.position.z = 0
    wp0.yaw = 0
    wp0.velocity = velocity
    traj.waypoints.append(wp0)

    wp1 = WaypointXYZVYaw()
    wp1.position.x = radius
    wp1.position.y = 0
    wp1.position.z = 0
    wp1.yaw = 0
    wp1.velocity = velocity
    traj.waypoints.append(wp1)

    for angle in np.arange(0, 2*np.pi, 10.*np.pi/180.):
        wp = WaypointXYZVYaw()
        wp.position.x = radius*np.cos(angle)
        wp.position.y = radius*np.sin(angle)
        wp.position.z = 0
        wp.yaw = 0
        wp.velocity = velocity
        traj.waypoints.append(wp)

    wp_end0 = WaypointXYZVYaw()
    wp_end0.position.x = radius
    wp_end0.position.y = 0
    wp_end0.position.z = 0
    wp_end0.yaw = 0
    wp_end0.velocity = velocity
    traj.waypoints.append(wp_end0)

    wp_end1 = WaypointXYZVYaw()
    wp_end1.position.x = 0
    wp_end1.position.y = 0
    wp_end1.position.z = 0
    wp_end1.yaw = 0
    wp_end1.velocity = velocity
    traj.waypoints.append(wp_end1)

    return traj

def interpolate_between_points(x1, y1, x2, y2, max_spacing):
    dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    num_points = 1 + int(np.ceil(dist/max_spacing))
    spacing = dist / (num_points - 1)

    path = []
    for i in range(num_points):
        x = x1 + (x2-x1) * spacing * i / dist
        y = y1 + (y2-y1) * spacing * i / dist
        path.append((x,y))
    
    return path

def interpolate_path(xy_path, max_spacing):
    assert len(xy_path) > 1
    interpolated_path = [xy_path[0]]
    for i in range(1, len(xy_path)):
        x1, y1 = xy_path[i-1]
        x2, y2 = xy_path[i]
        interpolated_path.extend(interpolate_between_points(x1, y1, x2, y2, max_spacing)[1:])
    return interpolated_path

def rotate_path(xy_path, angle_deg):
    # TODO: Vectorize using np array and mat multiplication
    angle_rad = np.deg2rad(angle_deg)
    rotated_xy_path = []
    for (x,y) in xy_path:
        new_x = x * np.cos(angle_rad) - y * np.sin(angle_rad)
        new_y = x * np.sin(angle_rad) + y * np.cos(angle_rad)
        rotated_xy_path.append((new_x, new_y))
    return rotated_xy_path

def get_rectangle_waypoints(attributes):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])

    path = [(0,0), (0, width), (length, width), (length, 0), (0, 0)]
    path = interpolate_path(path, max_spacing = 2.5)

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for (x,y) in path:
        wp1 = WaypointXYZVYaw()
        wp1.position.x = x
        wp1.position.y = y
        wp1.position.z = height
        wp1.yaw = 0
        wp1.velocity = velocity
        traj.waypoints.append(wp1)

    return traj


def get_horizontal_lawnmower_waypoints(attributes):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    stepover_dist = float(attributes['stepover_dist'])

    ccw_vertices = [(0, 0),(width, 0),(width, length),(0, length)]
    path = get_polygon_path(ccw_vertices, stepover_dist=stepover_dist)
    path.append((0,0))
    path = interpolate_path(path, max_spacing=2.5)

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    wp1 = WaypointXYZVYaw()
    wp1.position.x = 0
    wp1.position.y = 0
    wp1.position.z = height
    wp1.yaw = np.deg2rad(83.63349151611328+90.0)
    wp1.velocity = 3.0
    traj.waypoints.append(wp1)

    for (x,y) in path:
        wp1 = WaypointXYZVYaw()
        wp1.position.x = x
        wp1.position.y = y
        wp1.position.z = height
        wp1.yaw = np.deg2rad(83.63349151611328+90.0)
        wp1.velocity = velocity
        traj.waypoints.append(wp1)

    return traj


def fixed_trajectory_callback(msg):
    attributes = {}

    for key_value in msg.attributes:
        attributes[key_value.key] = key_value.value

    trajectory_msg = None

    if msg.type == 'Figure8':
        trajectory_msg = get_figure8_waypoints(attributes)
    elif msg.type == 'Circle':
        trajectory_msg = get_circle_waypoints(attributes)
    elif msg.type == 'Racetrack':
        trajectory_msg = get_racetrack_waypoints(attributes)
    elif msg.type == 'Line':
        trajectory_msg = get_line_waypoints(attributes)
    elif msg.type == 'Point':
        trajectory_msg = get_point_waypoints(attributes)
    elif msg.type == 'Rectangle':
        trajectory_msg = get_rectangle_waypoints(attributes)
    elif msg.type == 'Horizontal_Lawnmower':
        trajectory_msg = get_horizontal_lawnmower_waypoints(attributes)
        # trajectory_msg = get_coverage_waypoints(attributes)

    if trajectory_msg != None:
        trajectory_track_pub.publish(trajectory_msg)
    else:
        print('No trajectory sent.')

def convert_to_point2D(point32_list):
    pts = []
    for point32 in point32_list:
        p = Point2d(point32.x, point32.y)
        pts.append(p)

    return pts

def coverage_polygon_callback(msg):
    global polygonPointsList

    outerPoints = convert_to_point2D(msg.outerPolygon.points)
    polygonPointsList.append(outerPoints)

    holesList = []
    for hole in msg.holes:
        holesList.append(convert_to_point2D(hole))

    polygonPointsList.append(holesList)

def get_stepover_distance(height):
    cam_fov = 56
    non_overlapping_fov = 0.6 * cam_fov

    stepover = 2*np.tan(non_overlapping_fov/2)*height

    return stepover

def transform_odom(curr_odom, target_frame):

    tf_listener = TransformListener()

    if tf_listener.frameExists(curr_odom.header.frame_id) and tf_listener.frameExists(target_frame):
        curr_pose = PoseStamped()
        curr_pose.header.frame_id = curr_odom.header.frame_id
        curr_pose.pose = curr_odom.pose.pose

        transformed_pose = tf_listener.transformPose(target_frame, curr_pose)

        return (True, transformed_pose)
    else:
        rospy.logerr("Could not find transform between odom source frame and target map frame")
        return (False, None)

def get_extra_waypoint(curr_pose: PoseStamped, first_x, first_y, first_height, velocity):
    wp = WaypointXYZVYaw()
    wp.yaw = np.pi/2
    wp.velocity = velocity

    if curr_pose.pose.position.z > first_height:
        wp.position.x = first_x
        wp.position.y = first_y
        wp.position.z = curr_pose.pose.position.z
    else:
        wp.position.x = curr_pose.pose.position.x
        wp.position.y = curr_pose.pose.position.y
        wp.position.z = first_height

    return wp

def execute_coverage_planner_callback(msg):
    global polygonPointsList
    global current_odom

    if polygonPointsList:
        attributes = {}

        for key_value in msg.attributes:
            attributes[key_value.key] = key_value.value

        frame_id = str(attributes['frame_id'])
        height = float(attributes['height'])
        velocity = float(attributes['velocity'])

        global polygonPointsList
        outer_boundary, holes = polygonPointsList

        cells = trapezoidal_decomposition(outer_boundary, holes)
        cell_path = generate_cell_traversal(cells)
        stepover_dist = get_stepover_distance(height)

        path = get_full_coverage_path(cell_path, stepover_dist)
        path.append((0,0))
        path = interpolate_path(path, max_spacing=2.5)

        traj = TrajectoryXYZVYaw()
        traj.header.frame_id = frame_id

        success, transformed_curr_pose = transform_odom(current_odom, frame_id)
        if success:
            x, y = path[0]
            extra_wp = get_extra_waypoint(transformed_curr_pose, x, y, height, velocity)
            traj.waypoints.append(extra_wp)

            for (x,y) in path:
                wp1 = WaypointXYZVYaw()
                wp1.position.x = x
                wp1.position.y = y
                wp1.position.z = height
                wp1.yaw = np.pi/2
                wp1.velocity = velocity
                traj.waypoints.append(wp1)

            trajectory_track_pub.publish(traj)
        else:
            print('No trajectory sent.')

def odometry_callback(msg):
    global current_odom
    current_odom = msg

if __name__ == '__main__':
    rospy.init_node('fixed_trajectory_generator')

    fixed_trajectory_sub = rospy.Subscriber('fixed_trajectory', FixedTrajectory, fixed_trajectory_callback)

    trajectory_track_pub = rospy.Publisher('trajectory_track', TrajectoryXYZVYaw, queue_size=1)

    coverage_polygon_pub = rospy.Subscriber('coverage_polygon_points', PolygonArray, coverage_polygon_callback)

    odometry_sub = rospy.Subscriber('odometry', Odometry, odometry_callback)

    execute_coverage_planner_sub = rospy.Subscriber('execute_coverage_planner', FixedTrajectory, execute_coverage_planner_callback)

    rospy.spin()
