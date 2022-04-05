#include "firefly_panel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

namespace rviz {
    FireflyPanel::FireflyPanel(QWidget *parent)
            : rviz::Panel(parent) {
        QHBoxLayout *layout = new QHBoxLayout;
        clear_button_ = new QPushButton("Clear");
        set_local_pos_ref_button_ = new QPushButton("Set Reference");
        capture_frame_button_ = new QPushButton("Capture");
        layout->addWidget(clear_button_);
        layout->addWidget(set_local_pos_ref_button_);
        layout->addWidget(capture_frame_button_);
        setLayout(layout);

        connect(clear_button_, SIGNAL(clicked()), this, SLOT(clear()));
        connect(set_local_pos_ref_button_, SIGNAL(clicked()), this, SLOT(set_local_pos_ref()));
        connect(capture_frame_button_, SIGNAL(clicked()), this, SLOT(capture_frame()));

        clear_map_pub_ = nh_.advertise<std_msgs::Empty>("clear_map", 10);
        set_local_pos_ref_pub_ = nh_.advertise<std_msgs::Empty>("set_local_pos_ref", 10);
        capture_frame_pub_ = nh_.advertise<std_msgs::Empty>("capture_frame", 10);

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