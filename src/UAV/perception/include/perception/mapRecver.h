#ifndef MAP_RECV
#define MAP_RECV

#include <ros/ros.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/Layer.h>

namespace perception{
class MapRecver{
    public:
        MapRecver(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);


    private:


        void initRos();

        void MapCallback(const voxblox_msgs::Layer& layer_msg);

        bool map_class = true ; //default(true): esdf  
        ros::Subscriber map_sub_;

        //ros related

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        enum Map{ TSDF,ESDF};

        
};

}
#endif