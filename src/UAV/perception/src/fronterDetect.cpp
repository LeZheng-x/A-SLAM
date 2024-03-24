#include "perception/fronterDetect.h"

namespace perception
{
    Fronter::Fronter(){
        firstInitMap = true;
    }    
    
    bool Fronter::updateMap(const voxblox_msgs::Layer &voxelSets){
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
                //　查询是否存在index中blox是否存在,存在即进行更新，否则进行创建插入   
                std::shared_ptr<EsdfVoxel> oldBlockPtr = esdfMap_->getBlockPtrByIndex(index,true);// index信息
                
                //开始更新block数据
                 std::vector<uint32_t> data = block_msg.data;　//data信息
                 decodeBlockData(data,oldBlockPtr); 
            }
            // 合并
            else if(action == MapDerializationAction::kMerge){
                std::shared_ptr<EsdfVoxel> oldBlockPtr = esdfMap_->getBlockPtrByIndex(index,false);
                if(oldBlockPtr==nullptr){
                    std::cerr<<"nullptr prt !"<<std::endl;
                    return false;
                }else{ 
                    std::shared_ptr<EsdfVoxel> newBlockPtr = std::make_shared<EsdfVoxel>();
                    std::vector<uint32_t> data = block_msg.data;
                    decodeBlockData(data,newBlockPtr); // 合并的代码没有写 
                      if (newBlockPtr->observed && oldBlockPtr->observed) {
                        oldBlockPtr->distance = (newBlockPtr->distance + oldBlockPtr->distance) / 2.f;
                      } else if (newBlockPtr->observed && !oldBlockPtr->observed) {
                        oldBlockPtr->distance = newBlockPtr->distance;
                      }
                      oldBlockPtr->observed = oldBlockPtr->observed || newBlockPtr->observed;
                }
            }
        }
        return true;
    }

    void Fronter::decodeBlockData(const std::vector<uint32_t>& data,std::shared_ptr<EsdfVoxel> &block){
        constexpr size_t kNumDataPacketsPerVoxel = 2u;
        const size_t num_data_packets = data.size();
        for (size_t voxel_idx = 0u, data_idx = 0u;
            voxel_idx < esdfMap_->getNumVoxels() && data_idx < num_data_packets;
            ++voxel_idx, data_idx += kNumDataPacketsPerVoxel) {
            // Layout:
            // | 32 bit sdf | 3x8bit (int8_t) parent | 8 bit flags|

            const uint32_t bytes_1 = data[data_idx];
            const uint32_t bytes_2 = data[data_idx + 1u];

            memcpy(&(block->distance), &bytes_1, sizeof(bytes_1));

            block->observed = static_cast<bool>(bytes_2 & 0x00000001);
            block->hallucinated = static_cast<bool>((bytes_2 & 0x00000002));
            block->in_queue = static_cast<bool>((bytes_2 & 0x00000004));
            block->fixed = static_cast<bool>((bytes_2 & 0x00000008));

            // block->parent = deserializeDirection(bytes_2);
              Eigen::Vector3i parent_direction;

                parent_direction.x() = static_cast<int8_t>((bytes_2 >> 24) & 0x000000FF);
                parent_direction.y() = static_cast<int8_t>((bytes_2 >> 16) & 0x000000FF);
                parent_direction.z() = static_cast<int8_t>((bytes_2 >> 8) & 0x000000FF);
                block->parent = parent_direction;
        }
    }
} // namespace perception
