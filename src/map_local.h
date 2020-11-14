#ifndef local_map_cartesian_H
#define local_map_cartesian_H

#include <utils/include/all_utils.h>
#include <cell_cartesian.h>

class local_map_cartesian
{
private:
    int map_nx_times_ny;

    SE3 T_bs; //Transformation from sensor to body
    bool visibility_check;
    int measurement_cnt_max;
    int measurement_cnt_min;
    int occupied_measurement_cnt;
    int free_measurement_cnt;

public:
    double map_min_x;
    double map_min_y;
    double map_min_z;
    double map_dxyz;
    int map_nxy;
    int map_nz;

    vector<Vec3> visualization_cell_list;
    vector<Vec3I> occupied_cell_idx_list;
    vector<CARTESIAN_CELL> map;
    SE3 newest_T_wl;
    bool first_input;
    size_t mapIdx(Vec3I xyz_idx);
    size_t mapIdx(int x_idx, int y_idx, int z_idx);
    local_map_cartesian();
    void setTbs(SE3 T_bs_in);
    void init_map(double d_xyz,
                  int n_xy, int n_z,
                  int measurement_volume = 20,
                  double occupied_sh=0.7, double free_sh=-0.4);
    void creat_map();
    void creat_transfer_chart();
    Vec3I xyz2xyzIdx(Vec3 xyz_w);
    bool  xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx);
    void  input_pc_pose(vector<Vec3> PC_l, vector<Vec3> PC_miss_l, SE3 T_wl);

};

#endif // local_map_cartesian_H
