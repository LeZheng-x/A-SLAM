#include "perception/voxel.h"

namespace perception{

EsdfMap::EsdfMap(double voxel_size,uint32_t voxels_per_side):voxel_size_(voxel_size),voxels_per_side_(voxels_per_side){
    num_voxels_ = pow(voxels_per_side,3);
}

std::shared_ptr<EsdfVoxel> EsdfMap::getBlockPtrByIndex(Eigen::Vector3i index,bool ifUpdate = false){
        auto it = myMap.find(index);

        if(it == myMap.end()){//未找到即根据index创建新的block
            if(ifUpdate){
                 it =  myMap.emplace_hint(myMap.begin(),index,EsdfVoxel());
            }else{
                return nullptr;
            }
        }
        return std::make_shared<EsdfVoxel>(it->second);
}

}