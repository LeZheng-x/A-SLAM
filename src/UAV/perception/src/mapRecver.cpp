#include "perception/mapRecver.h"
namespace perception{

A_SLAM::A_SLAM(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private):nh_(nh),nh_private_(nh_private){
   initRos();                     
}

void A_SLAM::initRos(){

    if(map_class==Map::TSDF){
        map_sub_ = nh_private_.subscribe("tsdf_map_in", 1,&A_SLAM::MapCallback, this);
    }else if(map_class==Map::ESDF){
        map_sub_ = nh_private_.subscribe("esdf_map_in", 1,&A_SLAM::MapCallback, this);
    }
}         

void A_SLAM::MapCallback(const voxblox_msgs::Layer& layer_msg){ // 1 HZ
    voxelSets = layer_msg;
    setState(uint8_t(State::MAP_RECV));
}

}