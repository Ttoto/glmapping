#ifndef MAP_WAREHOUSE_H_
#define MAP_WAREHOUSE_H_

#include <utils/include/all_utils.h>
#include <map_local.h>


class map_warehouse
{
public:
    vector<SUBMAP> warehouse;

    void addSubMap();
    void deleteSubMap();
    bool searchSubMap(Vec3 center_xyz, double dis_sh, vector<unsigned int>& idx);


};

#endif // MAP_WAREHOUSE_H
