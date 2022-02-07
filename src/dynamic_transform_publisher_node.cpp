#include <ros/ros.h>
#include "dynamic_transform_publisher/dynamic_transform_publisher.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "dynamic_transform_publisher");
    
    DynamicTransformPublisher dynamic_transform_publisher;

    ros::spin();

    return 0;
}