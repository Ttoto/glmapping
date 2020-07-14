#ifndef CC_LOCAL_MAP_H
#define CC_LOCAL_MAP_H

#include <utils/include/all_utils.h>
#include <cc_agrid.h>
class cc_local_map
{
private:
    int nRho_x_nPhi;
    double z_border_min;
    double z_border_max;
    SE3 T_bs; //Transformation from sensor to body

public:

    double map_dRho;
    double map_dPhi;
    double map_dZ;
    int map_nRho;
    int map_nPhi;
    int map_nZ;
    int map_center_z_idx;

    vector<Vec3> visualization_grid_list;
    vector<cc_agrid> map;
    vector<cc_agrid> map_tmp;
    SE3 last_T_wl;
    bool first_input;
    size_t mapIdx(Vec3I Rho_Phi_z);
    size_t mapIdx(int Rho, int Phi, int z);
    cc_local_map();
    void setTbs(SE3 T_bs_in);
    void init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over);
    void creat_map();
    void creat_transfer_chart();
    Vec3I xyz2RhoPhiZ(Vec3 xyz_l);
    bool  xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz);
    void input_pc_pose(vector<Vec3> PC_s, SE3 T_wb);

};

#endif // CC_LOCAL_MAP_H
