#ifndef GLOBAL_MAP_CARTESIAN_H
#define GLOBAL_MAP_CARTESIAN_H

#include <utils/include/all_utils.h>
#include <cartesian_cell.h>

class global_map_cartesian
{
private:
    int map_nx_times_ny;
    double map_min_x;
    double map_min_y;
    double z_border_min;
    double z_border_max;
    SE3 T_bs; //Transformation from sensor to body
    bool visibility_check;

public:

    double map_dx;
    double map_dy;
    double map_dz;
    int map_nx;
    int map_ny;
    int map_nz;
    double map_min_z;

    vector<Vec3> visualization_cell_list;
    vector<cartesian_cell> map;
    SE3 last_T_wl;
    bool first_input;
    size_t mapIdx(Vec3I xyz_idx);
    size_t mapIdx(int x_idx, int y_idx, int z_idx);
    global_map_cartesian();
    void setTbs(SE3 T_bs_in);
    void init_map(double d_x, double d_y, double d_z, int n_x, int n_y, int n_z, double min_z);
    void creat_map();
    void creat_transfer_chart();
    Vec3I xyz2xyzIdx(Vec3 xyz_l);
    bool  xyz2xyzIdxwithBoderCheck(Vec3 xyz_l, Vec3I &xyz_idx);
    void  input_pc_pose(vector<Vec3> PC_l, SE3 T_wl);

};

#endif // GLOBAL_MAP_CARTESIAN_H
