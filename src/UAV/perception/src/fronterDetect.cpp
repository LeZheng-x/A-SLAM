#include "perception/fronterDetect.h"

namespace perception
{
    Fronter::Fronter(){
        firstInitMap = true;
    }    
    
    bool Fronter::decodeMapData(const voxblox_msgs::Layer &voxelSets){
        //１.获取地图的参数信息,初始化 or 更新
        auto action = static_cast<MapDerializationAction>(voxelSets.action);
        if(firstInitMap){
            esdfMap_ = std::make_shared<EsdfMap>(voxelSets.voxel_size,voxelSets.voxels_per_side);

            firstInitMap = false;
        }

        constexpr double kVoxelSizeEpsilon = 1e-5;
        if (voxelSets.voxels_per_side != esdfMap_->getVoxelPerSide() ||
            std::abs(voxelSets.voxel_size - esdfMap_->getVoxelSize()) > kVoxelSizeEpsilon) {
            std::cerr<< "Sizes don't match!"<<std::endl;
            return false;
        }
        
        if ( action== MapDerializationAction::kReset){
            esdfMap_->resetMap();
        }

        // //2.获取地图的区块(block)信息
        for (const voxblox_msgs::Block& block_msg : voxelSets.blocks) {
            Eigen::Vector3i index(block_msg.x_index,block_msg.y_index,block_msg.z_index); 

            //　更新　or 创建
            if (action == MapDerializationAction::kUpdate || !esdfMap_->hasBlock(index)) {

            }
            // 合并
            else if(action == MapDerializationAction::kMerge){

            }
        }

        return true;
    }
} // namespace perception
