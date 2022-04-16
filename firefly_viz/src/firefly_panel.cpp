#include "firefly_panel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

QLabel *battery = new QLabel;
QLabel *camera_status = new QLabel;
QLabel *temperature = new QLabel;
QLabel *altitude = new QLabel;

float base_station_altitude_;

void battery_status_gcs_callback(std_msgs::Float32 msg);
void temperature_status_gcs_callback(std_msgs::Float32 msg);
void camera_health_gcs_callback(std_msgs::Bool msg);
void altitude_status_gcs_callback(std_msgs::Float32 msg);
void base_station_altitude_gcs_callback(sensor_msgs::NavSatFix msg);

namespace rviz {
    FireflyPanel::FireflyPanel(QWidget *parent)
            : rviz::Panel(parent) {
        
        // setStyleSheet( "QWidget{ background-color : rgba(255,128,20, 100); border-radius : 7px;  }" );
        // setStyleSheet( "QGridLayout{ background-color : rgba( 255,128, 0, 125);}");

        QGridLayout *layout = new QGridLayout;

        // define all buttons for GUI
        clear_button_ = new QPushButton("Clear");
        set_local_pos_ref_button_ = new QPushButton("Set Reference");
        capture_frame_button_ = new QPushButton("Capture");
        ros_record_button_ = new QPushButton("ROS Bag Record");
        ros_stop_record_button_ = new QPushButton("Stop ROS Bag Recording");

        ///Initialize variables for fields
        QLabel *battery_status_text = new QLabel;
        QLabel *camera_status_text = new QLabel;
        QLabel *temperature_status_text = new QLabel;
        QLabel *altitude_status_text = new QLabel;

        //Define initial values for variables
        battery->setText("-9999");
        camera_status->setText("Waiting for Update");
        temperature->setText("-9999");
        altitude->setText("-9999");

        //Define values for fields
        battery_status_text->setText("Battery Level : ");
        camera_status_text->setText("Camera Status : ");
        temperature_status_text->setText("Temperature : ");
        altitude_status_text->setText("Altitude : ");

        //Buttons layout
        layout->addWidget(clear_button_,0,0);
        layout->addWidget(set_local_pos_ref_button_,0,1);
        layout->addWidget(capture_frame_button_,0,2);
        layout->addWidget(ros_record_button_,1,0);
        layout->addWidget(ros_stop_record_button_,1,1);

        //Update layout
        layout->addWidget(battery_status_text, 2, 0);
        layout->addWidget(battery, 2, 1);
        layout->addWidget(camera_status_text, 3, 0);
        layout->addWidget(camera_status, 3, 1);
        layout->addWidget(temperature_status_text, 4, 0);
        layout->addWidget(temperature, 4, 1);
        layout->addWidget(altitude_status_text, 5, 0);
        layout->addWidget(altitude, 4, 1);
        

        setLayout(layout);

        // Setting up actions for buttons
        connect(clear_button_, SIGNAL(clicked()), this, SLOT(clear()));
        connect(set_local_pos_ref_button_, SIGNAL(clicked()), this, SLOT(set_local_pos_ref()));
        connect(capture_frame_button_, SIGNAL(clicked()), this, SLOT(capture_frame()));
        connect(ros_record_button_, SIGNAL(clicked()), this, SLOT(record_ros_bag()));
        connect(ros_stop_record_button_, SIGNAL(clicked()), this, SLOT(stop_record_ros_bag()));

        //Publishers from GUI for Telemetry to read from
        clear_map_pub_ = nh_.advertise<std_msgs::Empty>("clear_map", 10);
        set_local_pos_ref_pub_ = nh_.advertise<std_msgs::Empty>("set_local_pos_ref", 10);
        capture_frame_pub_ = nh_.advertise<std_msgs::Empty>("capture_frame", 10);
        ros_record_ = nh_.advertise<std_msgs::Empty>("record_rosbag", 10);
        stop_ros_record_ = nh_.advertise<std_msgs::Empty>("stop_record_rosbag", 10);

        //Subscribers for GUI from Telemetry
        camera_health_gcs_ = nh_.subscribe("/camera_health_telem", 10, camera_health_gcs_callback);
        battery_status_gcs_ = nh_.subscribe("/battery_status_telem", 10, battery_status_gcs_callback);
        temperature_status_gcs_ = nh_.subscribe("/temperature_status_telem", 10, temperature_status_gcs_callback);
        altitude_status_gcs_ = nh_.subscribe("/altitude_status_telem", 10, altitude_status_gcs_callback);
        base_station_altitude_gcs_ = nh_.subscribe("/local_pos_ref", 10, base_station_altitude_gcs_callback);
    }

    void FireflyPanel::clear() {
        clear_map_pub_.publish(std_msgs::Empty());
    }

    void FireflyPanel::set_local_pos_ref() {
        set_local_pos_ref_pub_.publish(std_msgs::Empty());
    }

    void FireflyPanel::capture_frame() {
        capture_frame_pub_.publish(std_msgs::Empty());
    }

    void FireflyPanel::record_ros_bag() {
        ros_record_.publish(std_msgs::Empty());
    }

    void FireflyPanel::stop_record_ros_bag() {
        stop_ros_record_.publish(std_msgs::Empty());        
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
    if (msg.data)
        camera_status->setText("Working");
    else
        camera_status->setText("NOT Working");
}

void temperature_status_gcs_callback(std_msgs::Float32 msg) {
   if (msg.data)
        temperature->setText(QString::number(msg.data));
}

void altitude_status_gcs_callback(std_msgs::Float32 msg) {
   if (msg.data)
        altitude->setText(QString::number(msg.data - base_station_altitude_));
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


//1. change gcs telem to include battery and temperature
//2. MAVLINK msg type update
//3. update onboard_telem.py