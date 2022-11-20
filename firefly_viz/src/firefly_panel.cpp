#include "firefly_panel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

QLabel *battery = new QLabel;
QLabel *camera_status = new QLabel;
QLabel *altitude = new QLabel;
QLabel *detection_accuracy = new QLabel;
QLabel *association_accuracy = new QLabel;

float base_station_altitude_;
bool capturing{true};
bool runningMission{true};
bool mapping_terrain{true};

void battery_status_gcs_callback(std_msgs::Float32 msg);
void camera_health_gcs_callback(std_msgs::Bool msg);
void altitude_status_gcs_callback(std_msgs::Float32 msg);
void base_station_altitude_gcs_callback(sensor_msgs::NavSatFix msg);
void detection_accuracy_gcs_callback(std_msgs::Float32 msg);
void association_accuracy_gcs_callback(std_msgs::Float32 msg);

namespace rviz {
    FireflyPanel::FireflyPanel(QWidget *parent)
            : rviz::Panel(parent) {
        
        // setStyleSheet( "QWidget{ background-color : rgba(255,128,20, 100); border-radius : 7px;  }" );
        // setStyleSheet( "QGridLayout{ background-color : rgba( 255,128, 0, 125);}");

        QGridLayout *layout = new QGridLayout;

        // define all buttons for GUI
        arm_button_ = new QPushButton("ARM");
        disarm_button_ = new QPushButton("DISARM");
        clear_button_ = new QPushButton("Clear Map");
        set_local_pos_ref_button_ = new QPushButton("Set Reference");
        capture_frame_button_ = new QPushButton("Capture Frame");
        ros_record_button_ = new QPushButton("ROS Bag Record");
        ros_stop_record_button_ = new QPushButton("Stop ROS Bag Recording");
        view_coverage_poly_button_ = new QPushButton("Load Coverage Polygon");
        send_coverage_poly_button_ = new QPushButton("Send Coverage Polygon");
        request_control_button_ = new QPushButton("Request Control");
        takeoff_button_ = new QPushButton("TAKEOFF");
        land_button_ = new QPushButton("LAND");
        traj_control_button_ = new QPushButton("Traj Control");
        coverage_planner_button_ = new QPushButton("Coverage Planner");
        idle_button_ = new QPushButton("IDLE");
        terrain_mapping_button_ = new QPushButton("Start Terrain Map");
        reset_BT_button_ = new QPushButton("RESET BT");

        //define color for disarm
        QPalette pal_disarm = disarm_button_->palette();
        pal_disarm.setColor(QPalette::Button, QColor(Qt::red));
        disarm_button_->setAutoFillBackground(true);
        disarm_button_->setPalette(pal_disarm);
        disarm_button_->update();

        //define color for start mission button
        QPalette pal_arm = arm_button_->palette();
        pal_arm.setColor(QPalette::Button, QColor(Qt::green));
        arm_button_->setAutoFillBackground(true);
        arm_button_->setPalette(pal_arm);
        arm_button_->update();

        //define color for takeoff
        QPalette pal_takeoff = takeoff_button_->palette();
        pal_takeoff.setColor(QPalette::Button, QColor(Qt::blue));
        takeoff_button_->setAutoFillBackground(true);
        takeoff_button_->setPalette(pal_takeoff);
        takeoff_button_->update();

        //define color for land
        QPalette pal_land = land_button_->palette();
        pal_land.setColor(QPalette::Button, QColor(Qt::yellow));
        land_button_->setAutoFillBackground(true);
        land_button_->setPalette(pal_land);
        land_button_->update();

        ///Initialize variables for fields
        QLabel *battery_status_text = new QLabel;
        QLabel *camera_status_text = new QLabel;
        QLabel *altitude_status_text = new QLabel;
        QLabel *detection_accuracy_status_text = new QLabel;
        QLabel *association_accuracy_status_text = new QLabel;

        //Define initial values for variables
        battery->setText("-9999");
        camera_status->setText("Waiting for Update");
        altitude->setText("-9999");
        detection_accuracy->setText("0");
        association_accuracy->setText("0");

        //Define values for fields
        battery_status_text->setText("Battery Level : ");
        camera_status_text->setText("Camera Status : ");
        altitude_status_text->setText("Altitude : ");
        detection_accuracy_status_text->setText("Detection Accuracy : ");
        association_accuracy_status_text->setText("Association Accuracy : ");

        //Buttons layout
        layout->addWidget(arm_button_, 0, 1, 1, 2);
        layout->addWidget(clear_button_,1 , 0);
        layout->addWidget(set_local_pos_ref_button_,1 ,1);
        layout->addWidget(capture_frame_button_,1 , 2);
        layout->addWidget(ros_record_button_,2 ,0);
        layout->addWidget(ros_stop_record_button_,2 ,1);
        layout->addWidget(view_coverage_poly_button_,4 ,2);
        layout->addWidget(send_coverage_poly_button_,4 ,3);
        layout->addWidget(request_control_button_,0,0);
        layout->addWidget(takeoff_button_,0,3);
        layout->addWidget(land_button_,1,3,1,1);
        layout->addWidget(disarm_button_, 9, 0, 1, 4);
        layout->addWidget(traj_control_button_,3,2);
        layout->addWidget(coverage_planner_button_,5,2);
        layout->addWidget(idle_button_,2,3);
        layout->addWidget(terrain_mapping_button_,2,2);
        layout->addWidget(reset_BT_button_,3,3);

        //Update layout
        layout->addWidget(battery_status_text, 3, 0);
        layout->addWidget(battery, 3, 1);
        layout->addWidget(camera_status_text, 4, 0);
        layout->addWidget(camera_status, 4, 1);
        layout->addWidget(altitude_status_text, 5, 0);
        layout->addWidget(altitude, 5, 1);
        layout->addWidget(detection_accuracy_status_text, 6, 0);
        layout->addWidget(detection_accuracy, 6, 1);
        layout->addWidget(association_accuracy_status_text, 7, 0);
        layout->addWidget(association_accuracy, 7, 1);



        setLayout(layout);

        // Setting up actions for buttons
        connect(arm_button_, SIGNAL(clicked()), this, SLOT(arm())); 
        connect(disarm_button_, SIGNAL(clicked()), this, SLOT(disarm()));
        connect(clear_button_, SIGNAL(clicked()), this, SLOT(clear()));
        connect(set_local_pos_ref_button_, SIGNAL(clicked()), this, SLOT(set_local_pos_ref()));
        connect(capture_frame_button_, SIGNAL(clicked()), this, SLOT(capture_frame()));
        connect(ros_record_button_, SIGNAL(clicked()), this, SLOT(record_ros_bag()));
        connect(ros_stop_record_button_, SIGNAL(clicked()), this, SLOT(stop_record_ros_bag()));
        connect(view_coverage_poly_button_, SIGNAL(clicked()), this, SLOT(display_coverage_polygon()));
        connect(send_coverage_poly_button_, SIGNAL(clicked()), this, SLOT(send_coverage_polygon()));
        connect(request_control_button_, SIGNAL(clicked()), this, SLOT(request_control()));
        connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(takeoff()));
        connect(land_button_, SIGNAL(clicked()), this, SLOT(land()));
        connect(traj_control_button_, SIGNAL(clicked()), this, SLOT(traj_control()));
        connect(coverage_planner_button_, SIGNAL(clicked()), this, SLOT(coverage_planner()));
        connect(idle_button_, SIGNAL(clicked()), this, SLOT(idle()));
        connect(terrain_mapping_button_, SIGNAL(clicked()), this, SLOT(terrain_mapping()));
        connect(reset_BT_button_, SIGNAL(clicked()), this, SLOT(reset_BT()));

        //Publishers from GUI for Telemetry to read from
        arm_pub_ = nh_.advertise<std_msgs::Empty>("arm", 10);
        disarm_pub_ = nh_.advertise<std_msgs::Empty>("disarm", 10);
        clear_map_pub_ = nh_.advertise<std_msgs::Empty>("clear_map", 10);
        set_local_pos_ref_pub_ = nh_.advertise<std_msgs::Empty>("set_local_pos_ref", 10);
        capture_frame_pub_ = nh_.advertise<std_msgs::Empty>("capture_frame", 10);
        ros_record_ = nh_.advertise<std_msgs::Empty>("record_rosbag", 10);
        stop_ros_record_ = nh_.advertise<std_msgs::Empty>("stop_record_rosbag", 10);
        coverage_poly_view = nh_.advertise<std_msgs::Empty>("view_coverage_poly", 10);
        coverage_poly_send = nh_.advertise<std_msgs::Empty>("send_coverage_poly", 10);
        request_control_send = nh_.advertise<std_msgs::Empty>("request_control", 10);
        takeoff_send = nh_.advertise<std_msgs::Empty>("takeoff", 10);
        land_send = nh_.advertise<std_msgs::Empty>("land", 10);
        traj_control_send = nh_.advertise<std_msgs::Empty>("traj_control", 10);
        coverage_planner_send = nh_.advertise<std_msgs::Empty>("coverage_planner", 10);
        idle_send = nh_.advertise<std_msgs::Empty>("idle", 10);
        terrain_mapping_start_send = nh_.advertise<std_msgs::Empty>("start_terrain_mapping", 10);
        terrain_mapping_stop_send = nh_.advertise<std_msgs::Empty>("stop_terrain_mapping", 10);
        reset_BT_send = nh_.advertise<std_msgs::Empty>("reset_behavior_tree", 10);

        //Subscribers for GUI from Telemetry
        camera_health_gcs_ = nh_.subscribe("/camera_health_telem", 10, camera_health_gcs_callback);
        battery_status_gcs_ = nh_.subscribe("/battery_status_telem", 10, battery_status_gcs_callback);
        altitude_status_gcs_ = nh_.subscribe("/altitude_telem", 10, altitude_status_gcs_callback);
        base_station_altitude_gcs_ = nh_.subscribe("/local_pos_ref", 10, base_station_altitude_gcs_callback);
        detection_accuracy_gcs = nh_.subscribe("/detection_accuracy", 10,  detection_accuracy_gcs_callback);
        association_accuracy_gcs = nh_.subscribe("/association_accuracy", 10, association_accuracy_gcs_callback);
    }

    void FireflyPanel::arm() {
        arm_pub_.publish(std_msgs::Empty());
    }

    void FireflyPanel::disarm() {
        disarm_pub_.publish(std_msgs::Empty());
    }

    void FireflyPanel::clear() {
        clear_map_pub_.publish(std_msgs::Empty());

        association_accuracy->setText("0");
        detection_accuracy->setText("0");
    }

    void FireflyPanel::set_local_pos_ref() {
        set_local_pos_ref_pub_.publish(std_msgs::Empty());
    }

    void FireflyPanel::capture_frame() {
        capture_frame_pub_.publish(std_msgs::Empty());

        if (capturing){
            capture_frame_button_->setText("Stop Capture");
            capturing = false;
        }
        else{
            capturing = true;
            capture_frame_button_->setText("Capture");
        }        
    }
    void FireflyPanel::record_ros_bag() {
        ros_record_.publish(std_msgs::Empty());
    
        ros_record_button_->setText("ROS Bag Recording");
        ros_record_button_->setEnabled(false);
    }

    void FireflyPanel::stop_record_ros_bag() {
        stop_ros_record_.publish(std_msgs::Empty()); 

        ros_record_button_->setText("ROS Bag Record");

        ros_record_button_->setEnabled(true);
    }
    void FireflyPanel::display_coverage_polygon() {
        coverage_poly_view.publish(std_msgs::Empty());
    }
    void FireflyPanel::send_coverage_polygon() {
        coverage_poly_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::request_control(){
        request_control_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::takeoff(){
        takeoff_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::land(){
        land_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::traj_control(){
        traj_control_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::coverage_planner(){
        coverage_planner_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::idle(){
        idle_send.publish(std_msgs::Empty());
    }
    void FireflyPanel::terrain_mapping(){
        if(mapping_terrain){
            terrain_mapping_start_send.publish(std_msgs::Empty());
            mapping_terrain = false;
            terrain_mapping_button_->setText("Stop Terrain Map");
        }
        else{
            terrain_mapping_stop_send.publish(std_msgs::Empty());
            mapping_terrain = true;
            terrain_mapping_button_->setText("Start Terrain Map");
        }
    }
    void FireflyPanel::reset_BT(){
        reset_BT_send.publish(std_msgs::Empty());
    }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
    void FireflyPanel::save(rviz::Config config) const {
        rviz::Panel::save(config);
    }

// Load all configuration data for this panel from the given Config object.
    void FireflyPanel::load(const rviz::Config &config) {
        rviz::Panel::load(config);
    }

}

// ROS callbacks
void battery_status_gcs_callback(std_msgs::Float32 msg) {
    if (msg.data)
        battery->setText(QString::number(msg.data));
}

void camera_health_gcs_callback(std_msgs::Bool msg) {
    std::cout<<"IN CAMERA HEALTH"<<std::endl;
    if (msg.data)
        camera_status->setText("Working");
    else
        camera_status->setText("NOT Working");
}

void altitude_status_gcs_callback(std_msgs::Float32 msg) {
    std::cout<<"IN ALTITUDE"<<std::endl;
   if (msg.data)
        altitude->setText(QString::number(msg.data - base_station_altitude_));
}

void detection_accuracy_gcs_callback(std_msgs::Float32 msg) {
    if (msg.data)
        detection_accuracy->setText(QString::number(msg.data));
}

void association_accuracy_gcs_callback(std_msgs::Float32 msg) {
    if (msg.data)
        association_accuracy->setText(QString::number(msg.data));
}


void base_station_altitude_gcs_callback(sensor_msgs::NavSatFix msg) {
    base_station_altitude_ = msg.altitude;
}
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::FireflyPanel,rviz::Panel )
// END_TUTORIAL