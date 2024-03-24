#ifndef FRONTER
#define FRONTER

#include <Eigen/Core>
#include <vector>
#include "voxel.h"
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/Layer.h>
namespace perception{



// template <typename voxelType>
class Fronter{
    public: 
        Fronter();

        //
        bool extractFrontier();

        //voxblox的地图格式是经过序列化的产物，需要进行解码
        //[return]: ifSucess
        bool updateMap(const voxblox_msgs::Layer &voxelSets);

        void decodeBlockData(const std::vector<uint32_t>& data,std::shared_ptr<EsdfVoxel> &block);
    private:

      //本质上是根据ros接口消息去构建自己的esdf 
      std::shared_ptr<EsdfMap> esdfMap_;
      bool firstInitMap;
};



}


#endif