/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <keyBoard_Control/control.h>
// #include <nav_msgs/Odometry.h>
keyBoard_Control::control controlData;
geometry_msgs::Pose cntPose;
bool beginContol = false;

void controlCallback(const keyBoard_Control::control::ConstPtr& msg)
{
    //  ROS_INFO("Started hovering example.");
     controlData = *msg;
     beginContol  =true ;
}

void odomtryCallback(const geometry_msgs::Pose::ConstPtr& msg){
    cntPose = *msg;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ros::Subscriber subKeyBoard = nh.subscribe<keyBoard_Control::control>("keyBoard_Control", 500, &controlCallback);
  ros::Subscriber subPose = nh.subscribe<geometry_msgs::Pose>("/firefly/odometry_sensor1/pose", 500, &odomtryCallback);

  ros::Rate rate(150);  // control hz :150
  double ResolutionLength = 0.5 ; // 控制的分辨率 单位:m
  double ResolutionTheta = 0.087 ; // 控制的分辨率 单位:弧度
  double fixHeight = 1; //固定飞机高度

  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
  
  double desired_yaw = 0;
  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  //根据接受到的指令信号，参考当前机器人的位置，重新进行publish
  bool firstRun = true;

  while(ros::ok){

    ros::spinOnce();
    if(firstRun){
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
      trajectory_pub.publish(trajectory_msg);
      firstRun = false;
    }

    if(beginContol){
      //desired_position :  x y z 
      //desired_yaw : yaw
      //controlData：控制指令 q w e a s d p
      //cntPose ：当前位姿
      //ResolutionLength : 移动分辨率 单位:m
      //ResolutionTheta : 旋转分辨率 单位:弧度
      float Quaternion[4];
      Quaternion[0]=cntPose.orientation.w;
      Quaternion[1]=cntPose.orientation.x;
      Quaternion[2]=cntPose.orientation.y;
      Quaternion[3]=cntPose.orientation.z;
      double cntYaw = atan2(2*(Quaternion[0]*Quaternion[3]+Quaternion[1]*Quaternion[2]),1-2*(pow(Quaternion[2],2)+pow(Quaternion[3],2)));
      
      desired_position.z() = fixHeight;
      
      if(controlData.w_ ){ //前进 
           desired_position.x() = cntPose.position.x + cos(cntYaw)*ResolutionLength;
           desired_position.y() = cntPose.position.y + sin(cntYaw)*ResolutionLength;
           desired_yaw = cntYaw;
      }else if(controlData.s_ ){ //后退 
           desired_position.x() = cntPose.position.x - cos(cntYaw)*ResolutionLength;
           desired_position.y() = cntPose.position.y - sin(cntYaw)*ResolutionLength;
           desired_yaw = cntYaw;
      }else if(controlData.a_){ //向左
           desired_position.x() = cntPose.position.x - sin(cntYaw)*ResolutionLength;
           desired_position.y() = cntPose.position.y + cos(cntYaw)*ResolutionLength;
           desired_yaw = cntYaw;
      }else if(controlData.d_){ //向右
           desired_position.x() = cntPose.position.x + sin(cntYaw)*ResolutionLength;
           desired_position.y() = cntPose.position.y - cos(cntYaw)*ResolutionLength;
           desired_yaw = cntYaw;
      }else if(controlData.q_){//左转
          desired_position.x() = cntPose.position.x;
          desired_position.y() = cntPose.position.y;
          desired_yaw = cntYaw + ResolutionTheta;
      }else if(controlData.e_){//右转
          desired_position.x() = cntPose.position.x;
          desired_position.y() = cntPose.position.y;
          desired_yaw = cntYaw - ResolutionTheta;
      }
      else{ //暂停
          desired_position.x() = cntPose.position.x;
          desired_position.y() = cntPose.position.y;
          // desired_position.z() = cntPose.position.z;
          desired_yaw = cntYaw;
      }

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
      trajectory_pub.publish(trajectory_msg);
      beginContol = false;
    }


    rate.sleep();
  }

  ros::shutdown();

  return 0;
}
