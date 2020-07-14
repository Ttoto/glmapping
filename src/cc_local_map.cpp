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
    this->z_border_min = -(n_z_below*d_Z);
    this->nRho_x_nPhi = map_nRho*map_nPhi;
    //    cout  << "here" << endl;
    cout  << map_nRho << endl;
    cout  << map_nPhi << endl;
    cout  << map_nZ << endl;
    this->creat_map();
    map_tmp = map;
    first_input = true;
}

void cc_local_map::creat_map(void)
{
    for (int z=0; z<this->map_nZ; z++)
    {
        for (int phi=0; phi<this->map_nPhi; phi++)
        {
            for (int rho=0; rho<this->map_nRho; rho++)
            {
                double vis_z = this->z_border_min+(this->map_dZ/2)+(z*this->map_dZ);
                double vis_rho = this->map_dRho/2+(rho*this->map_dRho);
                double vis_phi = this->map_dPhi/2+(phi*this->map_dPhi);
                double vis_x = vis_rho*cos(vis_phi);
                double vis_y = vis_rho*sin(vis_phi);
                this->map.push_back(cc_agrid(rho,phi,z,Vec3(vis_x,vis_y,vis_z)));
            }
        }
    }
    cout << map.size() << endl;
    cc_agrid cell = map.at(this->mapIdx(Vec3I(7,3,8)));
}

Vec3I cc_local_map::xyz2RhoPhiZ(Vec3 xyz_l)
{
    double rho = sqrt(pow(xyz_l(0),2)+pow(xyz_l(1),2));
    int rho_idx =  static_cast<int>(rho/this->map_dRho);
    double phi = atan2(xyz_l(1),xyz_l(0));
    if (phi<0) phi += 2*M_PI;
    int phi_idx = static_cast<int>(phi/this->map_dPhi);
    double z = xyz_l(2)-this->z_border_min;
    int z_idx = static_cast<int>(z/this->map_dZ);
    return Vec3I(rho_idx,phi_idx,z_idx);
}

bool cc_local_map::xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz)
{
    double rho = sqrt(pow(xyz_l(0),2)+pow(xyz_l(1),2));
    int rho_idx =  static_cast<int>(rho/this->map_dRho);
    double phi = atan2(xyz_l(1),xyz_l(0));
    if (phi<0) phi += 2*M_PI;
    int phi_idx = static_cast<int>(phi/this->map_dPhi);
    double z = xyz_l(2)-this->z_border_min;
    int z_idx = static_cast<int>(z/this->map_dZ);
    rhophiz = Vec3I(rho_idx,phi_idx,z_idx);
    if(     rho_idx>0
            && phi_idx>0
            && z_idx>0
            && rho_idx < this->map_nRho
            && phi_idx < this->map_nPhi
            && z_idx < this->map_nZ)
    {
        return true;
    }
    return false;
}

void cc_local_map::input_pc_pose(vector<Vec3> PC_s, SE3 T_wb)
{
    //Frame [w]orld, [s]ensor, [b]ody, [l]ocalmap;
    cout << PC_s.size() << endl;
    SE3 T_wl(SO3(Quaterniond(1,0,0,0)),T_wb.translation());
    SE3 T_ws = T_wb * this->T_bs;
    SE3 T_ls = T_wl.inverse() * T_ws;
    vector<Vec3> PC_l;
    for(auto p_s:PC_s)
    {
        PC_l.push_back(T_ls*p_s);
    }
    //STEP 1: Transfer the local map.
    //    for(auto& grid:this->map)
    //    {
    //        grid.is_occupied = false;
    //    }

    if(first_input)
    {
        //do nothing
    }else
    {
        map_tmp = map;
        for(auto& grid:this->map)
        {
            grid.is_occupied = false;
        }
        for(auto& grid:this->map)
        {
            if(grid.is_occupied)
            {
                T_wl.translation()-
                        //    for(auto& grid:this->map)
                        //    {
                        //        grid.is_occupied = false;
                        //    }
                        //move the sample pt;
            }
        }
    }

    //STEP 2: Add measurement
    for(auto p_l:PC_l)
    {
        Vec3I rpz_idx;
        if(xyz2RhoPhiZwithBoderCheck(p_l,rpz_idx))
        {//set observerable
            map.at(this->mapIdx(rpz_idx)).is_occupied = true;
            map.at(this->mapIdx(rpz_idx)).sampled_xyz = p_l;
        }else
        {
        }
    }
    //Update occupied list;
    visualization_grid_list.clear();
    for(auto grid:this->map)
    {
        if(grid.is_occupied)
        {
            visualization_grid_list.push_back(grid.vis_pt);
        }
    }
    this->last_T_wl = T_wl;
    cout << "vis size" << visualization_grid_list.size() << endl;

}