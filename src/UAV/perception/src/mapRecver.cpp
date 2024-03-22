#include "perception/mapRecver.h"
namespace perception{

MapRecver::MapRecver(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private):nh_(nh),nh_private_(nh_private){
   initRos();                     
}

void MapRecver::initRos(){

    if(map_class==Map::TSDF){
        map_sub_ = nh_private_.subscribe("tsdf_map_in", 1,&MapRecver::MapCallback, this);
    }else if(map_class==Map::ESDF){
        map_sub_ = nh_private_.subscribe("esdf_map_in", 1,&MapRecver::MapCallback, this);
    }
}         

void MapRecver::MapCallback(const voxblox_msgs::Layer& layer_msg){
    ROS_INFO("RECED");
}
}