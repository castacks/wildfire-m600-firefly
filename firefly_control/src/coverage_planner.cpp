// list of 2d points, height, v -> trajectoryxyzvyaw msg
#include "ros.h"
#include "geometry_msgs/Polygon.h"


double get_coverage_width(const double height){
    return 20;
}

void Coverage_Planner(const geometry_msgs::Polygon& region, const double height) {
    const auto width = get_coverage_width(height);
    
}
