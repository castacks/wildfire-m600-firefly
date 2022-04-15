#include "firefly_panel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

// #include "mynewwidget.h"

// #include <QHBoxLayout>
// #include <QLabel>

// MyNewWidget::MyNewWidget(QWidget *parent) :
//     QWidget(parent)
// {
//     setStyleSheet( "QWidget{ background-color : rgba( 160, 160, 160, 255); border-radius : 7px;  }" );
//     QLabel *label = new QLabel(this);
//     QHBoxLayout *layout = new QHBoxLayout();
//     label->setText("Random String");
//     layout->addWidget(label);
//     setLayout(layout);
// }

namespace rviz {
    FireflyPanel::FireflyPanel(QWidget *parent)
            : rviz::Panel(parent) {
        
        // setStyleSheet( "QWidget{ background-color : rgba(255,128,20, 100); border-radius : 7px;  }" );
        // setStyleSheet( "QGridLayout{ background-color : rgba( 255,128, 0, 125);}");

        QGridLayout *layout = new QGridLayout;

        clear_button_ = new QPushButton("Clear");
        set_local_pos_ref_button_ = new QPushButton("Set Reference");
        capture_frame_button_ = new QPushButton("Capture");
        ros_record_button_ = new QPushButton("ROS Bag Record");
        ros_stop_record_button_ = new QPushButton("Stop ROS Bag Recording");

        QLabel *battery = new QLabel;


        layout->addWidget(clear_button_,0,0);
        layout->addWidget(set_local_pos_ref_button_,0,1);
        layout->addWidget(capture_frame_button_,0,2);
        layout->addWidget(ros_record_button_,1,0);
        layout->addWidget(ros_stop_record_button_,1,1);
        // layout->addRow("Battery : " + battery)

        setLayout(layout);

        // QGridLayout *statusLayout = new QGridLayout;
        // formGroupBox = new QGroupBox(tr("Battery Status : "));
        // formGroupBox->setLayout(layout);
        

        connect(clear_button_, SIGNAL(clicked()), this, SLOT(clear()));
        connect(set_local_pos_ref_button_, SIGNAL(clicked()), this, SLOT(set_local_pos_ref()));
        connect(capture_frame_button_, SIGNAL(clicked()), this, SLOT(capture_frame()));
        connect(ros_record_button_, SIGNAL(clicked()), this, SLOT(record_ros_bag()));
        connect(ros_stop_record_button_, SIGNAL(clicked()), this, SLOT(stop_record_ros_bag()));

        clear_map_pub_ = nh_.advertise<std_msgs::Empty>("clear_map", 10);
        set_local_pos_ref_pub_ = nh_.advertise<std_msgs::Empty>("set_local_pos_ref", 10);
        capture_frame_pub_ = nh_.advertise<std_msgs::Empty>("capture_frame", 10);
        ros_record_ = nh_.advertise<std_msgs::Empty>("record_rosbag", 10);
        stop_ros_record_ = nh_.advertise<std_msgs::Empty>("stop_record_rosbag", 10);

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
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::FireflyPanel,rviz::Panel )
// END_TUTORIAL