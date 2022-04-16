//
// Created by kevin on 4/5/22.
//

#ifndef FIREFLY_VIZ_FIREFLY_PANEL_H
#define FIREFLY_VIZ_FIREFLY_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace rviz {


    class FireflyPanel : public rviz::Panel {
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
    Q_OBJECT
    public:
        // QWidget subclass constructors usually take a parent widget
        // parameter (which usually defaults to 0).  At the same time,
        // pluginlib::ClassLoader creates instances by calling the default
        // constructor (with no arguments).  Taking the parameter and giving
        // a default of 0 lets the default constructor work and also lets
        // someone using the class for something else to pass in a parent
        // widget as they normally would with Qt.
        FireflyPanel(QWidget *parent = 0);

        // Now we declare overrides of rviz::Panel functions for saving and
        // loading data from the config file.  Here the data is the
        // topic name.
        virtual void load(const rviz::Config &config);

        virtual void save(rviz::Config config) const;

        // Here we declare some internal slots.
    protected Q_SLOTS:

        void clear();

        void set_local_pos_ref();

        void capture_frame();

        void record_ros_bag();

        void stop_record_ros_bag();

    protected:


        QPushButton *clear_button_;
        QPushButton *set_local_pos_ref_button_;
        QPushButton *capture_frame_button_;
        QPushButton *ros_record_button_;
        QPushButton *ros_stop_record_button_;


        ros::Publisher clear_map_pub_;
        ros::Publisher set_local_pos_ref_pub_;
        ros::Publisher capture_frame_pub_;
        ros::Publisher ros_record_;
        ros::Publisher stop_ros_record_;
        
        ros::Subscriber camera_health_gcs_;
        ros::Subscriber battery_status_gcs_;
        ros::Subscriber temperature_status_gcs_;
        ros::Subscriber altitude_status_gcs_;

        ros::NodeHandle nh_;

    };
}


#endif //FIREFLY_VIZ_FIREFLY_PANEL_H
