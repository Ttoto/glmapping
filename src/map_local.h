#ifndef local_map_cartesian_H
#define local_map_cartesian_H

#include <utils/include/all_utils.h>

typedef struct CARTESIAN_CELL{
    unsigned int idx;
    unsigned int relevant_submap_idx;
    unsigned int idx_x;
    unsigned int idx_y;
    unsigned int idx_z;
    Vec3  center_pt;
    bool  is_occupied;
    float log_odds;
    float sd;
}CARTESIAN_CELL;

typedef struct SUBMAP_CELL{
    Vec3  center_pt;
    bool  is_occupied;
}SUBMAP_CELL;

typedef struct SUBMAP_INFO{
    unsigned idx;
    Vec3  offset_min_xyz;
    Vec3  center_xyz;
    unsigned int num_occupied_cell;
}SUBMAP_INFO;

typedef struct SUBMAP{
    SUBMAP_INFO submap_info;
    vector<SUBMAP_CELL> cells;
}SUBMAP;

typedef struct SUBMAP_SWITCHING_CHECK_LIST{
    unsigned int local_sub_map_idx;
    SUBMAP_INFO submap_info;
}SUBMAP_SWITCHING_CHECK_LIST;

class local_map_cartesian
{
private:
    unsigned int map_nx_times_ny;
    unsigned int unique_submap_idx;

    SE3 T_wl; //Transformation from local to world
    bool visibility_check;
    float log_odds_max;
    float log_odds_min;
    float log_odds_hit;
    float log_odds_miss;
    float log_odds_occupied_sh;
    bool first_pose;

    Vec3   center_xyz;
    double map_min_x;
    double map_min_y;
    double map_min_z;
    double map_dxyz;
    unsigned int map_nxy;
    unsigned int map_nz;
    unsigned int submap_nxy;
    unsigned int submap_nz;
    SUBMAP_SWITCHING_CHECK_LIST sub_map_switching_check_list[27];

public:
    vector<Vec3> visualization_cell_list;
    vector<Vec3I> occupied_cell_idx_list;
    vector<CARTESIAN_CELL> map;
    size_t mapIdx(Vec3I xyz_idx);
    size_t mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx);
    local_map_cartesian();
    void init_map(double d_xyz_in,
                  unsigned int n_xy_in,
                  unsigned n_z_in,
                  float log_odds_min_in,
                  float log_odds_max_in,
                  float log_odds_hit_in,
                  float log_odds_miss_in,
                  float log_odds_occupied_sh_in);
    Vec3I xyz2xyzIdx(Vec3 xyz_w);
    bool  xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx);
    void  input_pc_pose(vector<Vec3> PC_hit_a, vector<Vec3> PC_miss_a, SE3 T_wa);

};

#endif // local_map_cartesian_H
