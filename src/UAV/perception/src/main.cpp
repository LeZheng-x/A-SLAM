#include "perception/mapRecver.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "perception_node");

    // Fronter fronter;
    


    ros::init(argc, argv, "voxblox");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    perception::A_SLAM UAV_planner(nh, nh_private);

    ros::Rate loop_rate(200);
    while (ros::ok())
    {   
        ros::spinOnce();

        if(UAV_planner.getState()==uint8_t(perception::A_SLAM::State::MAP_RECV)){
                //寻找前沿点，PCA 降维,构建 Frontier Structure
                // 
            UAV_planner.setState(uint8_t(perception::A_SLAM::State::FRONTER_DETECT));
            UAV_planner.extractFrontier();


            
        }
        /* code */

        loop_rate.sleep();

    }
    

    
    return 0;
}