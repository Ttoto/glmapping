#ifndef CC_AGRID_H
#define CC_AGRID_H

#include <utils/include/all_utils.h>

class cc_agrid
{
public:
    int idx_rho;
    int idx_phi;
    int idx_z;
    Vec3 sampled_xyz;
    bool is_occupied;
    float_t probability;


    Vec3 vis_pt;
    double vis_radius;


    cc_agrid();
    cc_agrid(int Rho, int Phi, int Z, Vec3 vis_xyz);

    bool obervation_update(SE3 pose, Vec3 xyz_l);
    bool occlusion_update();
};

#endif // CC_AGRID_H
