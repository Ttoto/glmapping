#include "cc_local_map.h"

cc_local_map::cc_local_map()
{

}

inline size_t cc_local_map::mapIdx(Vec3I Rho_Phi_z)
{
    return this->mapIdx(Rho_Phi_z(0),Rho_Phi_z(1),Rho_Phi_z(2));
}
inline size_t cc_local_map::mapIdx(int Rho, int Phi, int z)
{
    return static_cast<size_t>(z*this->nRho_x_nPhi
                               +Phi*this->map_nRho
                               +Rho);
}

void cc_local_map::setTbs(SE3 T_bs_in)
{
    this->T_bs=T_bs_in;
}

void cc_local_map::init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over)
{
    this->map_dRho = d_Rho;
    this->map_dPhi = d_Phi_deg * M_PI / 180;
    this->map_dZ   = d_Z;

    this->map_nRho = n_Rho;
    this->map_nPhi = static_cast<int>(360/d_Phi_deg);
    this->map_nZ   = n_z_below+n_z_over;
    this->map_center_z_idx = n_z_below;
    this->nRho_x_nPhi = map_nRho*map_nPhi;
//    cout  << "here" << endl;
    cout  << map_nRho << endl;
    cout  << map_nPhi << endl;
    cout  << map_nZ << endl;
    this->creat_map();
}

void cc_local_map::creat_map(void)
{
    for (int z=0; z<this->map_nZ; z++)
    {
        for (int phi=0; phi<this->map_nPhi; phi++)
        {
            for (int rho=0; rho<this->map_nRho; rho++)
            {
                this->map.push_back(cc_agrid(rho,phi,z,Vec3(0,0,0)));
            }
        }
    }
    cout << map.size() << endl;
    cc_agrid cell = map.at(this->mapIdx(Vec3I(7,3,8)));
    cout << cell.idx_rho << endl;
    cout << cell.idx_phi << endl;
    cout << cell.idx_z << endl;
    cout << "should be 738" << endl;
}

void cc_local_map::input_pc_pose(vector<Vec3> pc, SE3 T_wb)
{

}
