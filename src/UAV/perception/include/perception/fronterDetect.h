#ifndef FRONTER_DETECT
#define FRONTER_DETECT

#include <ros/ros.h>
#include <iostream>

namespace perception{
    
    class Fronter{

    public:
        Fronter();

        ~Fronter();

        void getFronter();

    private:
        ros::NodeHandle n;

        ros::Subscriber esdf_serv ;


        /*
        @brief 订阅esdf地图
        */
        void esdfCallback();
        
        /*
        @brief 检测前沿点　
        根据ｅｓｄｆ地图，划分前沿点并进行实时更新，确定前沿点最佳覆盖点，为global palnner 服务
        */
        void fronterDetect();


    };
}





#endif