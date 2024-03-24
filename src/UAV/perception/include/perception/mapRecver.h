#ifndef ASLAM
#define ASLAM

#include <ros/ros.h>
// #include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/Layer.h>
#include "fronterDetect.h"

namespace perception{
class A_SLAM{
    public:
        A_SLAM(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        inline uint8_t getState() { return state;}
        inline void setState(uint8_t state_){ state = state_;} 
        
        
        void extractFrontier();
        void globalPlanner(); //TODO: add 主动回环
        void localPlanner(); // 寻找最佳覆盖的路径搜索
        void trajecyOpt();  //去做轨迹优化

        enum Map{ TSDF,ESDF,OCCUPY}; //三种地图格式
        enum State{MAP_RECV,FRONTER_DETECT}; //两种状态
    private:


        void initRos();

        void MapCallback(const voxblox_msgs::Layer& layer_msg);

        bool map_class = true ; //default(true): esdf  
        ros::Subscriber map_sub_;

        //ros related

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;



        voxblox_msgs::Layer voxelSets; // 体素集合
        uint8_t state ; //机器人运动状态
        
};

}
#endif