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
    this->submap_paras.submap_nxy = n_xy_in/3;
    this->submap_paras.submap_nz = n_z_in/3;
    if((map_nxy!=submap_paras.submap_nxy*3)||(map_nz!=submap_paras.submap_nz*3))
    {
        cout << "critical error" << "please check the config file and make sure nxyz=(2n+1)*3" << endl;
        while(1);
    }
    this->submap_paras.submap_diff_center_offset = Vec3((0.5*map_dxyz)+floor(this->submap_paras.submap_nxy/2.0)*map_dxyz,
                                                        (0.5*map_dxyz)+floor(this->submap_paras.submap_nxy/2.0)*map_dxyz,
                                                        (0.5*map_dxyz)+floor(this->submap_paras.submap_nz/2.0)*map_dxyz);
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


    cout << map.size() << endl;
    for(size_t i=0; i<27; i++)
    {
        sub_map_switching_check_list[i].local_sub_map_idx=static_cast<unsigned int>(i);
    }
    this->unique_submap_idx = 0;
    this->occupied_cell_idx.clear();

    this->vis_paras.map_maxz = (0.5*map_dxyz)+floor(map_nz/2.0)*map_dxyz;
    this->vis_paras.map_minz = -(0.5*map_dxyz)-floor(map_nz/2.0)*map_dxyz;
    this->vis_paras.map_size_xy = this->map_dxyz*this->map_nxy;
    this->vis_paras.map_size_z = this->map_dxyz*this->map_nz;
    this->vis_paras.cube_size_xyz = this->map_dxyz;
}

void local_map_cartesian::allocate_memory_for_local_map()
{
    unsigned int idx_in_order=0;
    double minxy,minz;
    minxy = - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
    minz = - (0.5*map_dxyz)-floor(map_nz/2.0)*map_dxyz;
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
                cell.relevant_submap_idx = (z/submap_paras.submap_nz)*9+(y/submap_paras.submap_nxy)*3+(x/submap_paras.submap_nxy);
                double center_x = minxy + (this->map_dxyz/2)+(x*this->map_dxyz);
                double center_y = minxy + (this->map_dxyz/2)+(y*this->map_dxyz);
                double center_z = minz + (this->map_dxyz/2)+(z*this->map_dxyz);
                cell.center_pt = Vec3(center_x,center_y,center_z);
                this->map.push_back(cell);
            }
        }
    }
}

void local_map_cartesian::input_pc_pose(vector<Vec3> PC_hit_a, vector<Vec3> PC_miss_a, SE3 T_wa)
{
    if(this->first_pose)
    {

        //center_xyz and min_xyz
        double c_x= round(T_wa.translation().x()/map_dxyz)*this->map_dxyz;
        double c_y= round(T_wa.translation().y()/map_dxyz)*this->map_dxyz;
        double c_z= round(T_wa.translation().z()/map_dxyz)*this->map_dxyz;
        this->map_center_xyz = Vec3(c_x,c_y,c_z);
        this->map_min_x = map_center_xyz(0) - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
        this->map_min_y = map_center_xyz(1) - (0.5*map_dxyz)-floor(map_nxy/2.0)*map_dxyz;
        this->map_min_z = map_center_xyz(2) - (0.5*map_dxyz)-floor(map_nz/2.0)*map_dxyz;
        this->map_min_xyz = Vec3(this->map_min_x,this->map_min_y,this->map_min_z);
        this->T_wl =  SE3(SO3(0,0,0),map_center_xyz);
        //init the switching check list
        Vec3 diff_min_center = this->map_min_xyz - this->map_center_xyz;
        cout << "map center: " << this->T_wl.translation().transpose() << " offset " << this->map_min_xyz.transpose() << endl;
        for(size_t i=0; i<27; i++)
        {
            sub_map_switching_check_list[i].submap_info.center_xyz=Vec3(
                        map_min_x + this->submap_paras.submap_diff_center_offset(0) + (i%3)*this->map_dxyz*this->submap_paras.submap_nxy,
                        map_min_y + this->submap_paras.submap_diff_center_offset(1) + ((i%9)/3)*this->map_dxyz*this->submap_paras.submap_nxy,
                        map_min_z + this->submap_paras.submap_diff_center_offset(2) + (i/9)*this->map_dxyz*this->submap_paras.submap_nz
                        );
            sub_map_switching_check_list[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
            cout << "submap idx: " << i << " center: " << sub_map_switching_check_list[i].submap_info.center_xyz.transpose()
                 << "  offset min: " << sub_map_switching_check_list[i].submap_info.offset_min_xyz.transpose() << endl;
        }
        first_pose = false;
    }else
    {
        //check whether need to swith to another localmap
        Vec3 t_wa=T_wa.translation();
        double min_distance_2_center = (t_wa-this->map_center_xyz).norm();
        double min_distance_2_other = 999.0;
        unsigned int min_distance_2_other_idx=27;
        for(size_t i=0; i<27; i++)
        {
            if(i==13)continue;
            double dis = (sub_map_switching_check_list[i].submap_info.center_xyz - t_wa).norm();
            if(dis<min_distance_2_other)
            {
                min_distance_2_other = dis;
                min_distance_2_other_idx = static_cast<unsigned int>(i);
            }
        }
        cout << "min_distance_2_center=" << min_distance_2_center << " while " << "min_distance_2_other=" << min_distance_2_other << endl;
        //if so, switch
        if(min_distance_2_center>min_distance_2_other)
        {
            cout << "min_distance_2_center=" << min_distance_2_center << " while " << "min_distance_2_other=" << min_distance_2_other << endl;
            cout << "switch to " << min_distance_2_other_idx << endl;
            for(auto& i:this->map)
            {
                i.is_occupied=false;
            }

            this->map_center_xyz = sub_map_switching_check_list[min_distance_2_other_idx].submap_info.center_xyz;
            this->map_min_xyz    = sub_map_switching_check_list[min_distance_2_other_idx].submap_info.offset_min_xyz;
            this->map_min_x = this->map_min_xyz(0);
            this->map_min_y = this->map_min_xyz(1);
            this->map_min_z = this->map_min_xyz(2);
            this->T_wl =  SE3(SO3(0,0,0),this->map_center_xyz);
            //init the switching check list
            Vec3 diff_min_center = this->map_min_xyz - this->map_center_xyz;
            cout << "map center: " << this->T_wl.translation().transpose() << " offset " << this->map_min_xyz.transpose() << endl;
            for(size_t i=0; i<27; i++)
            {
                sub_map_switching_check_list[i].submap_info.center_xyz=Vec3(
                            map_min_x + this->submap_paras.submap_diff_center_offset(0) + (i%3)*this->map_dxyz*this->submap_paras.submap_nxy,
                            map_min_y + this->submap_paras.submap_diff_center_offset(1) + ((i%9)/3)*this->map_dxyz*this->submap_paras.submap_nxy,
                            map_min_z + this->submap_paras.submap_diff_center_offset(2) + (i/9)*this->map_dxyz*this->submap_paras.submap_nz
                            );
                sub_map_switching_check_list[i].submap_info.offset_min_xyz = diff_min_center + sub_map_switching_check_list[i].submap_info.center_xyz;
                cout << "submap idx: " << i << " center: " << sub_map_switching_check_list[i].submap_info.center_xyz.transpose()
                     << "offset min: " << sub_map_switching_check_list[i].submap_info.offset_min_xyz.transpose() << endl;
            }
        }

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
    occupied_cell_idx.clear();
    for(auto cell:this->map)
    {
        if(cell.is_occupied)
        {
            occupied_cell_idx.push_back(cell.idx);
            //visualization_cell_list.push_back(cell.vis_pt);
        }
    }
    //cout << "globalmap vis size" << visualization_cell_list.size() << endl;

}
