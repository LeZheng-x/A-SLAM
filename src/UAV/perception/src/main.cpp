#include "perception/fronterDetect.h"
#include "perception/mapRecver.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "perception_node");

    // Fronter fronter;
    


    ros::init(argc, argv, "voxblox");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    perception::MapRecver node(nh, nh_private);

    ros::spin();
    return 0;
}