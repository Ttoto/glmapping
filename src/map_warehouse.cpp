#include "map_warehouse.h"



bool map_warehouse::searchSubMap(Vec3 center_xyz, double dis_sh, vector<unsigned int>& idx)
{
    bool ret=false;
    for (auto submap:this->warehouse) {
        if((submap.submap_info.center_xyz - center_xyz).norm() < dis_sh)
        {
            idx.push_back(submap.submap_info.idx);
            ret =  true;
        }
    }
    return ret;
}
