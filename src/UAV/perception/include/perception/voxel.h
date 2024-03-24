#ifndef VOXEL 
#define VOXEL

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <unordered_map>

namespace perception{

    enum class MapDerializationAction : uint8_t {
        kUpdate = 0u,
        kMerge = 1u,
        kReset = 2u
        };

    // 自定义哈希函数类
    struct EigenVector3Hash {
        std::size_t operator()(const Eigen::Vector3i& v) const {
            // 使用 Eigen 向量的数据进行哈希计算
            return std::hash<int>{}(v[0]) ^ (std::hash<int>{}(v[1]) << 1) ^ (std::hash<int>{}(v[2]) << 2);
        }
    };

    struct EsdfVoxel {
        float distance = 0.0f;

        bool observed = false;
        /**
         * Whether the voxel was copied from the TSDF (false) or created from a pose
         * or some other source (true). This member is not serialized!!!
         */
        bool hallucinated = false;
        bool in_queue = false;
        bool fixed = false;

        /**
         * Relative direction toward parent. If itself, then either uninitialized
         * or in the fixed frontier.
         */
        Eigen::Vector3i parent = Eigen::Vector3i::Zero();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    //EsdfVoxel 的集合体
    class EsdfMap {
        public:
            EsdfMap(double voxel_size,uint32_t voxels_per_side);
            inline double getVoxelSize(){return voxel_size_;}
            inline double getVoxelPerSide(){return voxels_per_side_;}

            //block采用hash map　形式进行储存，方便索引查询与新数据的插入
            inline void resetMap(){myMap.clear();}

            //根据Index获得Block的智能指针，根据ifUpdate选择:不存在则根据Index创建,返回空指针
            std::shared_ptr<EsdfVoxel> getBlockPtrByIndex(Eigen::Vector3i index,bool ifUpdate);
            
            //


            inline size_t getNumOfBlock(){return myMap.size();}
            inline bool hasBlock(const Eigen::Vector3i & index){
                return myMap.count(index)>0 ; 
            }
            inline size_t getNumVoxels(){return num_voxels_;}
        private:
            double voxel_size_ ; 
            uint32_t voxels_per_side_ ;
            size_t num_voxels_;
            //哈希表计算方法：　将位置信息 index 按位进行异或操作
            std::unordered_map<Eigen::Vector3i, EsdfVoxel, EigenVector3Hash> myMap;

    };

}

#endif