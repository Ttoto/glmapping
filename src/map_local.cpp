#include "map_local.h"

local_map_cartesian::local_map_cartesian()
{

}

size_t local_map_cartesian::mapIdx(Vec3I xyz_idx)
{
    return this->mapIdx(static_cast<unsigned int>(xyz_idx(0)),
                        static_cast<unsigned int>(xyz_idx(1)),
                        static_cast<unsigned int>(xyz_idx(2)));
}
size_t local_map_cartesian::mapIdx(unsigned int x_idx, unsigned int y_idx, unsigned int z_idx)
{
    return static_cast<size_t>( z_idx*this->map_nx_times_ny
                                +y_idx*this->map_dxyz
                                +x_idx);
}

Vec3I local_map_cartesian::xyz2xyzIdx(Vec3 xyz_w)
{
    double x = xyz_w(0)-this->map_min_x;
    int x_idx =  static_cast<int>(x/this->map_dxyz);
    double y = xyz_w(1)-this->map_min_y;
    int y_idx =  static_cast<int>(y/this->map_dxyz);
    double z = xyz_w(2)-this->map_min_z;
    int z_idx =  static_cast<int>(z/this->map_dxyz);
    return Vec3I(x_idx,y_idx,z_idx);
}

bool local_map_cartesian::xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx)
{
    double x = xyz_w(0)-this->map_min_x;
    int x_idx =  static_cast<int>(x/this->map_dxyz);
    double y = xyz_w(1)-this->map_min_y;
    int y_idx =  static_cast<int>(y/this->map_dxyz);
    double z = xyz_w(2)-this->map_min_z;
    int z_idx =  static_cast<int>(z/this->map_dxyz);
    if(x>0 && y>0 && z>0)
    {
        if( x>0 && y>0 && z>0
                && x_idx>=0 && y_idx>=0 && z_idx>=0
                && x_idx < static_cast<int>(this->map_nxy)
                && y_idx < static_cast<int>(this->map_nxy)
                && z_idx < static_cast<int>(this->map_nz))
        {
            xyz_idx = Vec3I(x_idx,y_idx,z_idx);
            return true;
        }
    }
    return false;
}

void local_map_cartesian::init_map(double d_xyz_in,
                                   unsigned int n_xy_in,
                                   unsigned n_z_in,
                                   float log_odds_min_in,
                                   float log_odds_max_in,
                                   float log_odds_hit_in,
                                   float log_odds_miss_in,
                                   float log_odds_occupied_sh_in)
{
    this->map_dxyz = d_xyz_in;
    this->map_nxy = n_xy_in;
    this->map_nz = n_z_in;
    this->submap_nxy = n_xy_in/3;
    this->submap_nz = n_z_in/3;
    if((map_nxy!=submap_nxy*3)||(map_nz!=submap_nz*3))
    {
        cout << "critical error" << "please check the config file and make sure nxyz=(2n+1)*3" << endl;
        while(1);
    }
    this->map_nx_times_ny = n_xy_in*n_xy_in;
    this->map_min_x = 0;
    this->map_min_y = 0;
    this->map_min_z = 0;    
    first_pose = true;

    this->log_odds_min = log_odds_min_in;
    this->log_odds_max = log_odds_max_in;
    this->log_odds_hit = log_odds_hit_in;
    this->log_odds_miss = log_odds_miss_in;
    this->log_odds_occupied_sh = log_odds_occupied_sh_in;

    unsigned int idx_in_order=0;
    for (unsigned int z=0; z<this->map_nz; z++)
    {
        for (unsigned int y=0; y<this->map_nxy; y++)
        {
            for (unsigned int x=0; x<this->map_nxy; x++)
            {
                CARTESIAN_CELL cell;
                cell.is_occupied = false;
                cell.sd = 0;
                cell.idx_x = x;
                cell.idx_y = y;
                cell.idx_z = z;
                cell.log_odds = 0;
                cell.idx = idx_in_order;
                cell.relevant_submap_idx = (z/submap_nz)*9+(y/submap_nxy)*3+(x/submap_nxy);
                double center_x = (this->map_dxyz/2)+(x*this->map_dxyz);
                double center_y = (this->map_dxyz/2)+(y*this->map_dxyz);
                double center_z = (this->map_dxyz/2)+(z*this->map_dxyz);
                cell.center_pt = Vec3(center_x,center_y,center_z);
                this->map.push_back(cell);
            }
        }
    }
    cout << map.size() << endl;
    for(size_t i=0; i<27; i++)
    {
        sub_map_switching_check_list[i].local_sub_map_idx=static_cast<unsigned int>(i);
    }
    this->unique_submap_idx = 0;
    this->occupied_cell_idx_list.clear();
}


void local_map_cartesian::input_pc_pose(vector<Vec3> PC_hit_a, vector<Vec3> PC_miss_a, SE3 T_wa)
{
    if(this->first_pose)
    {
        this->T_wl =  SE3(SO3(0,0,0),T_wa.translation());
        //center_xyz and min_xyz
        double c_x= round(T_wa.translation().x()/map_dxyz)*this->map_dxyz;
        double c_y= round(T_wa.translation().y()/map_dxyz)*this->map_dxyz;
        double c_z= round(T_wa.translation().z()/map_dxyz)*this->map_dxyz;
        this->center_xyz = Vec3(c_x,c_y,c_z);
        this->map_min_x = center_xyz(0) - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
        this->map_min_y = center_xyz(1) - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
        this->map_min_z = center_xyz(2) - (0.5*map_dxyz)-floor(map_nz/2.0)*map_dxyz;
        //init the switching check list
        for(size_t i=0; i<27; i++)
        {
            sub_map_switching_check_list[i].submap_info.center_xyz=Vec3(
                        map_min_x + (0.5*map_dxyz) + (i%3)*this->map_dxyz*this->submap_nxy,
                        map_min_y + (0.5*map_dxyz) + (i/3)*this->map_dxyz*this->submap_nz,
                        map_min_z + (0.5*map_dxyz) + (i/9)*this->map_dxyz*this->submap_nz
                        );
        }

        first_pose = false;
    }else
    {
        //check whether need to swith to another localmap

        //if so, switch

    }
    //update map.

    //Frame [w]orld, [s]ensor, [b]ody, [a]wareness, [l]ocalmap;
    vector<Vec3> pc_hit_w;
    vector<Vec3> pc_miss_w;
    for(auto p_a:PC_hit_a)
    {
        pc_hit_w.push_back(T_wa*p_a);
    }
    for(auto p_a:PC_miss_a)
    {
        pc_miss_w.push_back(T_wa*p_a);
    }
    //STEP 2: Add measurement
    for(auto p_w:pc_hit_w)
    {
        Vec3I xyz_idx;
        if(xyz2xyzIdxwithBoderCheck(p_w,xyz_idx))
        {
            size_t map_idx=mapIdx(xyz_idx);
            map.at(map_idx).log_odds+=this->log_odds_hit;
            if(map.at(map_idx).log_odds > this->log_odds_max)
            {
                map.at(map_idx).log_odds=log_odds_max;
            }
            //set observerable
            if(map.at(map_idx).log_odds>this->log_odds_occupied_sh){
                map.at(this->mapIdx(xyz_idx)).is_occupied = true;
                //map.at(this->mapIdx(xyz_idx)).sampled_xyz = p_w;
            }
        }else
        {
        }
    }
    for(auto p_miss_w:pc_miss_w)
    {
        Vec3I xyz_idx;
        if(xyz2xyzIdxwithBoderCheck(p_miss_w,xyz_idx))
        {
            size_t map_idx=mapIdx(xyz_idx);
            map.at(map_idx).log_odds--;
            if(map.at(map_idx).log_odds < this->log_odds_min)
            {
                map.at(map_idx).log_odds=log_odds_min;
            }
            //set free
            if(map.at(map_idx).log_odds < this->log_odds_occupied_sh){
                map.at(this->mapIdx(xyz_idx)).is_occupied = false;
            }
        }else
        {
        }
    }
    //Update occupied list;
    visualization_cell_list.clear();
    occupied_cell_idx_list.clear();
    for(auto cell:this->map)
    {
        if(cell.is_occupied)
        {
            occupied_cell_idx_list.push_back(Vec3I(cell.idx_x,cell.idx_y,cell.idx_z));
            //visualization_cell_list.push_back(cell.vis_pt);
        }
    }
    //cout << "globalmap vis size" << visualization_cell_list.size() << endl;

}
