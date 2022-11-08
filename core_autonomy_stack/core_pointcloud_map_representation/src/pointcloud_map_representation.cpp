#include <core_pointcloud_map_representation/pointcloud_map_representation.h>
#include <tflib/tflib.h>
#include <pluginlib/class_list_macros.h>
#include <limits>

PointCloudMapRepresentation::PointCloudMapRepresentation()//ros::NodeHandle* nh)
  : debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()){
  ros::NodeHandle* nh = new ros::NodeHandle();
  ros::NodeHandle* pnh = new ros::NodeHandle("~");
  
  // init params
  node_spacing = pnh->param("pointcloud_map/node_spacing", 1.0);
  node_limit = pnh->param("pointcloud_map/node_limit", 5);
  node_decay_time = pnh->param("pointcloud_map/node_decay_time", 30.);
  target_frame = pnh->param("pointcloud_map/target_frame", std::string("map"));
  lidar_frame = pnh->param("pointcloud_map/lidar_frame", std::string("/lidar_frame"));
  lidar_vertical_fov = M_PI/180.*pnh->param("pointcloud_map/lidar_vertical_fov", 15.0);
  robot_radius = pnh->param("pointcloud_map/robot_radius", 0.6);
  obstacle_check_points = pnh->param("pointcloud_map/obstacle_check_points", 5);
  obstacle_check_radius = pnh->param("pointcloud_map/obstacle_check_radius", 2.);

  should_add_node = false;
  got_odom = false;
  range_up_valid = false;
  range_down_valid = false;
  
  // init subscribers
  cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("uav1/points", 1, &PointCloudMapRepresentation::cloud_callback, this);
  odom_sub = nh->subscribe<nav_msgs::Odometry>("odometry", 1, &PointCloudMapRepresentation::odom_callback, this);
  range_up_sub = nh->subscribe<sensor_msgs::Range>("range_up", 1, &PointCloudMapRepresentation::range_up_callback, this);
  range_down_sub = nh->subscribe<sensor_msgs::Range>("range_down", 1, &PointCloudMapRepresentation::range_down_callback, this);
  listener = new tf::TransformListener();

  // init publishers
  debug_pub = nh->advertise<sensor_msgs::PointCloud2>("debug_points", 1);
  /*cloud_map_pub = nh->advertise<sensor_msgs::PointCloud2>("/debug_points", 1);
  debug_pub = nh->advertise<visualization_msgs::MarkerArray>("/pointcloud_map_debug", 1);
  */
  pose_graph_pub = nh->advertise<visualization_msgs::MarkerArray>("pointcloud_map/pose_graph", 1);
  
  // init debug marker attributes
  points.ns = "obstacles";
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.action = visualization_msgs::Marker::ADD;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.scale.z = 0.1;
}

std::vector< std::vector<double> > PointCloudMapRepresentation::get_values(std::vector<std::vector<geometry_msgs::PointStamped> > trajectories){//std::vector<core_trajectory_msgs::TrajectoryXYZVYaw> trajectories){
  std::vector< std::vector<double> > values(trajectories.size());

  for(int i = 0; i < trajectories.size(); i++)
    //for(int j = 0; j < trajectories[i].waypoints.size(); j++)
    for(int j = 0; j < trajectories[i].size(); j++)
      values[i].push_back(100);

  for(int i = 0; i < trajectories.size(); i++){
    //core_trajectory_msgs::TrajectoryXYZVYaw trajectory = trajectories[i];
    
    //for(int j = 0; j < trajectory.waypoints.size(); j++){
    for(int j = 0; j < trajectories[i].size(); j++){
      geometry_msgs::PointStamped point_stamped = trajectories[i][j];
      bool seen = false;
    
      tf::StampedTransform transform;
      //listener->waitForTransform(target_frame, trajectory.header.frame_id, trajectory.header.stamp, ros::Duration(0.1));
      //listener->lookupTransform(target_frame, trajectory.header.frame_id, trajectory.header.stamp, transform);
      listener->waitForTransform(target_frame, point_stamped.header.frame_id, point_stamped.header.stamp, ros::Duration(0.1));
      listener->lookupTransform(target_frame, point_stamped.header.frame_id, point_stamped.header.stamp, transform);
    
      //tf::Vector3 position = transform*tflib::to_tf(trajectory.waypoints[j].position);
      tf::Vector3 position = transform*tflib::to_tf(point_stamped.point);
      pcl::PointXYZ point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();

      double min_distance = std::numeric_limits<double>::max();

      std::vector<PointCloudNode> nodes = get_all_nodes();
      //ROS_INFO("--------------------------------------------------------------------------------");
      for(int k = 0; k < nodes.size(); k++){
	tf::Vector3 position_cloud_frame = nodes[k].target_to_lidar_frame_transform*position;
	double angle = position_cloud_frame.angle(tf::Vector3(position_cloud_frame.x(), position_cloud_frame.y(), 0));//atan2(position_cloud_frame.z(), position_cloud_frame.x());
	double distance = position_cloud_frame.length();
	//ROS_INFO_STREAM("angle: " << 180./M_PI*angle << " lidar v fov: " << 180./M_PI*lidar_vertical_fov/2. << " distance: " << distance);
	// if(distance <= robot_radius || fabs(angle) <= lidar_vertical_fov/2.)
  seen = true;
      
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree = nodes[k].kdtree;
      
	std::vector<int> indices;
	std::vector<float> squared_distances;
	if(kdtree == NULL)
	  return values;
	kdtree->nearestKSearch(point, 1, indices, squared_distances);
      
	if(squared_distances.size() > 0)
	  min_distance = std::min(min_distance, (double)sqrt(squared_distances[0]));

      }
    
      pcl::PointXYZRGB debug_point;
      debug_point.x = position.x();
      debug_point.y = position.y();
      debug_point.z = position.z();
      if(!seen){
	debug_point.r = 0;
	debug_point.g = 0;
	debug_point.b = 255;
      }
      else if(min_distance <= robot_radius){
	debug_point.r = 255;
	debug_point.g = 0;
	debug_point.b = 0;
      }
      else{
	debug_point.r = 0;
	debug_point.g = 255;
	debug_point.b = 0;
      }
    
      debug_cloud->points.push_back(debug_point);
    
      if(!seen)
	min_distance = 0;
    
      values[i][j] = min_distance;
    }
  }
  
  return values;
}

void PointCloudMapRepresentation::clear(){
  for(int i = 0; i < 20; i++)
    ROS_INFO("MAP CLEARED");
  node_buffer.clear();
}

double PointCloudMapRepresentation::distance_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction){
  try{
    bool seen = false;
    
    tf::StampedTransform transform;
    listener->waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, pose.header.frame_id, pose.header.stamp, transform);
    
    tf::Vector3 position = transform*tflib::to_tf(pose.pose.position);
    pcl::PointXYZ point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();

    double min_distance = std::numeric_limits<double>::max();

    std::vector<PointCloudNode> nodes = get_all_nodes();
    //ROS_INFO("--------------------------------------------------------------------------------");
    for(int i = 0; i < nodes.size(); i++){
      tf::Vector3 position_cloud_frame = nodes[i].target_to_lidar_frame_transform*position;
      double angle = position_cloud_frame.angle(tf::Vector3(position_cloud_frame.x(), position_cloud_frame.y(), 0));//atan2(position_cloud_frame.z(), position_cloud_frame.x());
      double distance = position_cloud_frame.length();
      //ROS_INFO_STREAM("angle: " << 180./M_PI*angle << " lidar v fov: " << 180./M_PI*lidar_vertical_fov/2. << " distance: " << distance);
      if(distance <= robot_radius || fabs(angle) <= lidar_vertical_fov/2.)
	seen = true;
      
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree = nodes[i].kdtree;
      
      std::vector<int> indices;
      std::vector<float> squared_distances;
      if(kdtree == NULL)
	return 0;
      kdtree->nearestKSearch(point, 1, indices, squared_distances);
      
      if(squared_distances.size() > 0)
	min_distance = std::min(min_distance, (double)sqrt(squared_distances[0]));

      /*
      if(i == 0){
	if(nodes[i].range_up_valid){
	  double xy_distance = sqrt(pow(nodes[i].range_point_up.x() - point.x, 2) + pow(nodes[i].range_point_up.y() - point.y, 2));
	  if(point.z >= nodes[i].range_point_up.z()){
	    if(xy_distance <= nodes[i].up_radius)
	      min_distance = 0;
	  }
	  else if(xy_distance <= nodes[i].up_radius){
	    min_distance = std::min(min_distance, (double)(nodes[i].range_point_up.z() - point.z));
	  }
	}
      }
      */
    }
    
    pcl::PointXYZRGB debug_point;
    debug_point.x = position.x();
    debug_point.y = position.y();
    debug_point.z = position.z();
    debug_point.r = seen ? 0 : 255;//(1.0 - std::min(min_distance, 10.0)/10.0)*255;
    debug_point.g = seen ? 255 : 0;
    debug_point.b = 0;
    
    debug_cloud->points.push_back(debug_point);
    
    if(!seen)
      min_distance = 0;
    
    return min_distance;
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform Exception in distance_to_obstacle while looking up tf from " << pose.header.frame_id << " to " << target_frame << ": " << ex.what());
  }
  
  return 0;
}

void PointCloudMapRepresentation::publish_debug(){
  /*
  for(double x = -2; x < 2; x += 0.1){
    for(double y = -2; y < 2; y += 0.1){
      for(double z = -2; z < 2; z += 0.1){
	ros::Time now = ros::Time::now()-ros::Duration(0.1);
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = now;
	pose.header.frame_id = "look_ahead_point_stabilized";
	pose.pose.orientation.w = 1;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	distance_to_obstacle(pose, tf::Vector3(0, 0, 0));
      }
    }
    ROS_INFO_STREAM("x: " << x);
  }
  //*/
    
  // publish debug pointcloud
  sensor_msgs::PointCloud2 debug_cloud2;
  pcl::toROSMsg(*debug_cloud, debug_cloud2);
  debug_cloud2.header.frame_id = target_frame;
  debug_pub.publish(debug_cloud2);
  
  debug_cloud->points.clear();

  // publish pose graph
  std::vector<PointCloudNode> nodes = get_all_nodes();
  //ROS_INFO_STREAM("nodes.size(): " << nodes.size());
  if(nodes.size() > 0){
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    for(int i = 0; i < nodes.size(); i++){
      nav_msgs::Odometry curr_odom = nodes[i].odom;
      
      visualization_msgs::Marker pose;
      pose.header.stamp = now;
      pose.header.frame_id = curr_odom.header.frame_id;
      pose.ns = "pointcloud_map_pose_graph";
      pose.id = i;
      pose.type = visualization_msgs::Marker::ARROW;
      pose.action = visualization_msgs::Marker::ADD;
      
      pose.pose.position.x = curr_odom.pose.pose.position.x;
      pose.pose.position.y = curr_odom.pose.pose.position.y;
      pose.pose.position.z = curr_odom.pose.pose.position.z;
      pose.pose.orientation.x = curr_odom.pose.pose.orientation.x;
      pose.pose.orientation.y = curr_odom.pose.pose.orientation.y;
      pose.pose.orientation.z = curr_odom.pose.pose.orientation.z;
      pose.pose.orientation.w = curr_odom.pose.pose.orientation.w;
      pose.scale.x = 0.5;
      pose.scale.y = 0.1;
      pose.scale.z = 0.1;
      float age = node_decay_time > 0 ? (now - curr_odom.header.stamp).toSec()/node_decay_time
	: 0.f;
      pose.color.r = age;
      pose.color.g = 1 - age;
      pose.color.b = 0;
      pose.color.a = 1;

      if(i == 0){
	visualization_msgs::Marker circle_up, circle_down;
	circle_up.header.stamp = circle_down.header.stamp = now;
	circle_up.header.frame_id = circle_down.header.frame_id = target_frame;
	circle_up.type = circle_down.type = visualization_msgs::Marker::SPHERE;
	circle_up.action = circle_down.action = visualization_msgs::Marker::ADD;
	circle_up.scale.z = circle_down.scale.z = 0.05;
	circle_up.color.r = circle_down.color.r = 0;
	circle_up.color.g = circle_down.color.g = 1;
	circle_up.color.b = circle_down.color.b = 0;
	circle_up.color.a = circle_down.color.a = .3;
      
	circle_up.ns = "upper_limit";
	circle_up.id = i;
	circle_up.scale.x = 2*nodes[i].up_radius;
	circle_up.scale.y = 2*nodes[i].up_radius;
	circle_up.pose.position.x = nodes[i].range_point_up.x();
	circle_up.pose.position.y = nodes[i].range_point_up.y();
	circle_up.pose.position.z = nodes[i].range_point_up.z();
	circle_up.pose.orientation.w = 1;
      
	circle_down.ns = "lower_limit";
	circle_down.id = i;
	circle_down.scale.x = 2*nodes[i].down_radius;
	circle_down.scale.y = 2*nodes[i].down_radius;
	circle_down.pose.position.x = nodes[i].range_point_down.x();
	circle_down.pose.position.y = nodes[i].range_point_down.y();
	circle_down.pose.position.z = nodes[i].range_point_down.z();
	circle_down.pose.orientation.w = 1;

	if(nodes[i].range_up_valid)
	  marker_array.markers.push_back(circle_up);
	if(nodes[i].range_down_valid)
	  marker_array.markers.push_back(circle_down);
      }
      
      marker_array.markers.push_back(pose);
    }
    
    pose_graph_pub.publish(marker_array);
  }
}


void PointCloudMapRepresentation::cloud_callback(sensor_msgs::PointCloud2 cloud){
  if(!got_odom)
    return;
  
  try{
    sensor_msgs::PointCloud2 cloud_target_frame;
    tf::StampedTransform lidar_to_target_frame_transform;
    listener->waitForTransform(target_frame, "uav1/map", cloud.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, "uav1/map", cloud.header.stamp, lidar_to_target_frame_transform);
    //ROS_INFO_STREAM("transformPointCloud: target_frame " << target_frame << " cloud frame " << cloud.header.frame_id);
    pcl_ros::transformPointCloud(target_frame, cloud, cloud_target_frame, *listener);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_target_frame, *current_cloud);
    
    if(current_cloud->points.size() == 0)
      return;
    
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    //ROS_INFO("ACCESSING KDTREE 1");
    if(kdtree == NULL)
      return;
    kdtree->setInputCloud(current_cloud);
    //ROS_INFO("ACCESSING KDTREE 1 DONE");
    
    for(int i = 0; i < current_cloud->points.size(); i++){
      pcl::PointXYZ point = current_cloud->points[i];
      
      pcl::PointXYZRGB p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      p.r = 0;
      p.g = 255;
      p.b = 0;
      
      debug_cloud->points.push_back(p);
    }
    
    current_node.kdtree = kdtree;
    current_node.odom = odom;
    current_node.cloud = current_cloud;
    current_node.target_to_lidar_frame_transform = lidar_to_target_frame_transform.inverse();
    
    current_node.range_point_up = range_point_up;
    current_node.range_up_valid = range_up_valid && (fabs((range_up_time - odom.header.stamp).toSec()) < 0.5);
    current_node.up_radius = up_radius;
    
    current_node.range_point_down = range_point_down;
    current_node.range_down_valid = range_down_valid && (fabs((range_down_time - odom.header.stamp).toSec()) < 0.5);
    current_node.down_radius = down_radius;
    
    if(should_add_node || node_buffer.size() == 0){
      node_buffer.push_back(current_node);
    }
    
    if(node_buffer.size() > node_limit){
      node_buffer.erase(node_buffer.begin());
    }
    
    should_add_node = false;
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform Exception in cloud_callback: " << ex.what());
  }
}

void PointCloudMapRepresentation::odom_callback(nav_msgs::Odometry odom){
  got_odom = true;
  this->odom = odom;
  
  if(node_buffer.size() > 0){
    double distance = tflib::to_tf(odom.pose.pose.position).distance(tflib::to_tf(node_buffer[node_buffer.size()-1].odom.pose.pose.position));
    if(distance >= node_spacing)
      should_add_node = true;
  }
  else
    should_add_node = true;
}

void PointCloudMapRepresentation::range_up_callback(sensor_msgs::Range range){
  /*
  tf::Vector3 point_range_frame(range.range, 0, 0);
  range_up_valid = get_point_in_frame(target_frame, range.header.frame_id, range.header.stamp, point_range_frame, &range_point_up);
  range_up_time = range.header.stamp;

  tf::Vector3 point_lidar_frame;
  bool lidar_point_valid = get_point_in_frame(lidar_frame, range.header.frame_id, range.header.stamp, point_range_frame, &point_lidar_frame);
  if(lidar_point_valid){
    up_radius = point_lidar_frame.z()/tan(0.5*lidar_vertical_fov);
  }
  else
    range_up_valid = false;
  */
}

void PointCloudMapRepresentation::range_down_callback(sensor_msgs::Range range){
  /*
  tf::Vector3 point_range_frame(range.range, 0, 0);
  range_down_valid = get_point_in_frame(target_frame, range.header.frame_id, range.header.stamp, point_range_frame, &range_point_down);
  range_down_time = range.header.stamp;
  
  tf::Vector3 point_lidar_frame;
  bool lidar_point_valid = get_point_in_frame(lidar_frame, range.header.frame_id, range.header.stamp, point_range_frame, &point_lidar_frame);
  if(lidar_point_valid){
    down_radius = -point_lidar_frame.z()/tan(0.5*lidar_vertical_fov);
  }
  else
    range_up_valid = false;
  */
}

bool PointCloudMapRepresentation::get_point_in_frame(std::string target_frame, std::string frame_id, ros::Time time, tf::Vector3 point_range_frame, tf::Vector3* point_target_frame){
  try{
    tf::StampedTransform transform;
    listener->waitForTransform(target_frame, frame_id, time, ros::Duration(0.1));
    listener->lookupTransform(target_frame, frame_id, time, transform);
    
    *point_target_frame = transform*point_range_frame;
    
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform Exception in cloud_callback: " << ex.what());
    return false;
  }

  return true;
}

std::vector<PointCloudNode> PointCloudMapRepresentation::get_all_nodes(){
  ros::Time now = ros::Time::now();
  
  std::vector<PointCloudNode> nodes;
  nodes.push_back(current_node);
  for(int i = 0; i < node_buffer.size(); i++)
    if(node_decay_time <= 0.f || (now - node_buffer[i].odom.header.stamp).toSec() <= node_decay_time)
      nodes.push_back(node_buffer[i]);
  return nodes;
}











// UNTESTED
tf::Vector3 PointCloudMapRepresentation::distance_vector_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction){
  try{
    tf::StampedTransform transform;
    listener->waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, pose.header.frame_id, pose.header.stamp, transform);
    
    tf::Vector3 position = transform*tflib::to_tf(pose.pose.position);
    pcl::PointXYZ point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();

    double min_distance_x = std::numeric_limits<double>::max();
    double min_distance_y = std::numeric_limits<double>::max();
    double min_distance_z = std::numeric_limits<double>::max();

    std::vector<PointCloudNode> nodes = get_all_nodes();
    for(int i = 0; i < nodes.size(); i++){
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree = nodes[i].kdtree;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = nodes[i].cloud;
      
      std::vector<int> indices;
      std::vector<float> squared_distances;
      if(kdtree == NULL)
	return tf::Vector3(0, 0, 0);
      kdtree->radiusSearch(point, robot_radius, indices, squared_distances);
      
      for(int j = 0; j < indices.size(); j++){
	pcl::PointXYZ p = cloud->points[indices[j]];

	double x_dist = sqrt(pow(p.x - point.x, 2));
	double y_dist = sqrt(pow(p.y - point.y, 2));
	double z_dist = sqrt(pow(p.z - point.z, 2));

	min_distance_x = std::min(min_distance_x, x_dist);
	min_distance_y = std::min(min_distance_y, y_dist);
	min_distance_z = std::min(min_distance_z, z_dist);
      }
    }
    
    pcl::PointXYZRGB debug_point;
    debug_point.x = position.x();
    debug_point.y = position.y();
    debug_point.z = position.z();
    debug_point.r = 255;
    debug_point.g = 0;
    debug_point.b = 0;
    
    debug_cloud->points.push_back(debug_point);

    return tf::Vector3(min_distance_x, min_distance_y, min_distance_z);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform Exception in distance_to_obstacle while looking up tf from " << pose.header.frame_id << " to " << target_frame << ": " << ex.what());
  }
  
  return tf::Vector3(0, 0, 0);
}


/*
double PointCloudMapRepresentation::distance_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction){
  try{
    tf::StampedTransform transform;
    listener->waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, pose.header.frame_id, pose.header.stamp, transform);
    
    points.header.frame_id = target_frame;//pose.header.frame_id;
    std_msgs::ColorRGBA green;
    green.r = 0;
    green.b = 0;
    green.g = 1;
    green.a = 1;
    std_msgs::ColorRGBA red;
    red.r = 1;
    red.b = 0;
    red.g = 0;
    red.a = 1;


    tf::Quaternion q_up, q_down, q_left, q_right;
    q_up.setRPY(0, -M_PI/2, 0);
    q_down.setRPY(0, M_PI/2, 0);
    q_left.setRPY(0, 0, M_PI/2);
    q_right.setRPY(0, 0, -M_PI/2);
    std::vector<tf::Quaternion> directions;
    directions.push_back(q_up);
    directions.push_back(q_down);
    directions.push_back(q_left);
    directions.push_back(q_right);
    
    tf::Vector3 position = transform*tflib::to_tf(pose.pose.position);
    tf::Quaternion q = transform*tflib::to_tf(pose.pose.orientation);
    tf::Vector3 unit(1, 0, 0);
    double closest_obstacle_distance = obstacle_check_radius;
  
    direction.normalize();
    tf::Vector3 up = tf::Transform(q_up*q)*unit;
    up.normalize();

    tf::Vector3 side = up.cross(direction);
    side.normalize();

    up = side.cross(direction);
    up.normalize();

    std::vector<tf::Vector3> direction_vectors;
    direction_vectors.push_back(up);
    direction_vectors.push_back(side);
    direction_vectors.push_back(-up);
    direction_vectors.push_back(-side);

    std::vector<PointCloudNode> nodes = get_all_nodes();
    for(int i = 0; i < nodes.size(); i++){
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree = nodes[i].kdtree;
    
      for(int m = 0; m < directions.size(); m++){
	tf::Quaternion q_curr = directions[m];
	for(int k = 1; k < obstacle_check_points+1; k++){
	  double dist = (double)k*obstacle_check_radius/(double)(obstacle_check_points+1);
      
	  tf::Vector3 point = position + dist*direction_vectors[m];//tf::Transform(q_curr)*(dist*direction);
	  geometry_msgs::PoseStamped check_pose;
	  check_pose.header = pose.header;
	  check_pose.pose.position.x = point.x();
	  check_pose.pose.position.y = point.y();
	  check_pose.pose.position.z = point.z();
	  check_pose.pose.orientation.w = 1.0;
	  pcl::PointXYZ pcl_point;
	  pcl_point.x = point.x();
	  pcl_point.y = point.y();
	  pcl_point.z = point.z();

	  // check for collision
	  bool collision = false;
	  std::vector<int> indices;
	  std::vector<float> distances;
	  if(kdtree == NULL)
	    return 0;
	  kdtree->nearestKSearch(pcl_point, 1, indices, distances);
	  if(distances.size() > 0 && distances[0] <= robot_radius)
	    collision = true;
      
	  std_msgs::ColorRGBA c;
	  if(collision){
	    closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
	    c = red;
	  }
	  else
	    c = green;
      
	  points.points.push_back(check_pose.pose.position);
	  points.colors.push_back(c);
	}
      }
      
      points.points.push_back(pose.pose.position);
      points.colors.push_back(green);

      pcl::PointXYZ pcl_point;
      pcl_point.x = position.x();
      pcl_point.y = position.y();
      pcl_point.z = position.z();

      // check for collision
      bool collision = false;
      std::vector<int> indices;
      std::vector<float> distances;
      if(kdtree == NULL)
	return 0;
      kdtree->nearestKSearch(pcl_point, 1, indices, distances);
      if(distances.size() > 0 && distances[0] <= robot_radius)
	collision = true;
      if(collision)
	return 0;
    }
    
    return closest_obstacle_distance;
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("Transform Exception in distance_to_obstacle while looking up tf from " << pose.header.frame_id << " to " << target_frame << ": " << ex.what());
  }
  
  return 0;
}

void PointCloudMapRepresentation::publish_debug(){
  // publish collision checks
  points.header.stamp = ros::Time::now();
  markers.markers.push_back(points);

  debug_pub.publish(markers);

  markers.markers.clear();
  points.points.clear();
  points.colors.clear();

  // publish pose graph
  std::vector<PointCloudNode> nodes = get_all_nodes();
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_clouds(new pcl::PointCloud<pcl::PointXYZ>());
  if(nodes.size() > 0){
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    for(int i = 0; i < nodes.size(); i++){
      nav_msgs::Odometry curr_odom = nodes[i].odom;
      all_clouds->points.insert(all_clouds->points.begin(), nodes[i].cloud->points.begin(), nodes[i].cloud->points.end());
      
      visualization_msgs::Marker pose;
      pose.header.stamp = now;
      pose.header.frame_id = curr_odom.header.frame_id;
      pose.ns = "pointcloud_map_pose_graph";
      pose.id = i;
      pose.type = visualization_msgs::Marker::ARROW;
      pose.action = visualization_msgs::Marker::ADD;

      pose.pose.position.x = curr_odom.pose.pose.position.x;
      pose.pose.position.y = curr_odom.pose.pose.position.y;
      pose.pose.position.z = curr_odom.pose.pose.position.z;
      pose.pose.orientation.x = curr_odom.pose.pose.orientation.x;
      pose.pose.orientation.y = curr_odom.pose.pose.orientation.y;
      pose.pose.orientation.z = curr_odom.pose.pose.orientation.z;
      pose.pose.orientation.w = curr_odom.pose.pose.orientation.w;
      pose.scale.x = 0.5;
      pose.scale.y = 0.1;
      pose.scale.z = 0.1;
      pose.color.r = 0;
      pose.color.g = 1;
      pose.color.b = 0;
      pose.color.a = 1;

      marker_array.markers.push_back(pose);
    }
    
    pose_graph_pub.publish(marker_array);
  }

  
  // publish debug pointcloud
  sensor_msgs::PointCloud2 debug_cloud2;
  pcl::toROSMsg(*all_clouds, debug_cloud2);
  debug_cloud2.header.frame_id = target_frame;
  cloud_map_pub.publish(debug_cloud2);
}
*/

PLUGINLIB_EXPORT_CLASS(PointCloudMapRepresentation, MapRepresentation)
