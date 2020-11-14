#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <mlmapping/local2global.h>
#include <map_cartesian.h>
#include <msg_awareness2local.h>
#include <rviz_vis.h>
//#include <global2occupancygrid2d.h>
//#include <global2esdf.h>
//#include <global2esdf3d.h>

namespace mlmapping_ns
{

class LocalMapNodeletClass : public nodelet::Nodelet
{
public:
    LocalMapNodeletClass()  {;}
    ~LocalMapNodeletClass() {;}
private:

    ros::Subscriber sub_from_local;
    local_map_cartesian* global_map;
    rviz_vis *globalmap_publisher;
//    Global2OccupancyGrid2D *occupancy_grid_publisher;
//    Global2ESDF *esfd2d_publisher;
//    Global2ESDF3DPatch *esfd3d_publisher;
    ros::Time last_esft_stamp;

    void from_lm_callback(const mlmapping::local2globalConstPtr& msg)
    {
        SE3 T_wl;
        vector<Vec3> l2g_obs_l;
        vector<Vec3> l2g_miss_l;
        ros::Time stamp;
        msg_awareness2local::unpack(msg,T_wl,l2g_obs_l,l2g_miss_l,stamp);
        global_map->input_pc_pose(l2g_obs_l,l2g_miss_l,T_wl);
        globalmap_publisher->pub_globalmap(global_map->visualization_cell_list,stamp);
//        occupancy_grid_publisher->pub_occupancy_grid_2D_from_globalmap(*global_map,stamp);
//        esfd3d_publisher->pub_ESDF_3D_from_globalmap(*global_map,stamp);
//        if((ros::Time::now().toSec()-last_esft_stamp.toSec())>0.19)
//        {
//            esfd2d_publisher->pub_ESDF_2D_from_globalmap(*global_map,stamp);
//        }
    }

    virtual void onInit()
    {
        cout << "globalmapnode:" << endl;
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        cout << "this is mlmapping global node" << endl;
        nh.getParam("/mlmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;

        double d_xyz  = getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_d_xyz");
        int    n_xy  = getIntVariableFromYaml(configFilePath,"mlmapping_gm_n_xy");
        int    n_z  = getIntVariableFromYaml(configFilePath,"mlmapping_gm_n_z");
        double log_odd_min = getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_log_odds_max");
        double log_odd_max = getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_log_odds_max");
        double measurement_hit = getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_measurement_hit");
        double measurement_miss = getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_measurement_miss");
        double occupied_sh = getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_occupied_sh");

        global_map = new local_map_cartesian();
//        global_map->init_map(d_x,d_y,d_z,n_x,n_y,n_z,min_z,
//                             measure_cnt,occupied_sh);
//        double max_z=min_z+(d_z*n_z);
        globalmap_publisher =  new rviz_vis();
//        globalmap_publisher->set_as_globalmap_publisher(nh,"/globalmap","map",2,min_z,max_z,d_x,d_z);
//        occupancy_grid_publisher = new Global2OccupancyGrid2D(nh,"/occupancygrid",2);
//        occupancy_grid_publisher->setGlobalMap(*global_map,"map");
//        esfd2d_publisher = new Global2ESDF(nh,"/esfd_map",2);
//        esfd2d_publisher->setGlobalMap(*global_map,"map");
//        esfd3d_publisher = new Global2ESDF3DPatch(nh,"/esfd_batch",2);
//        esfd3d_publisher->setGlobalMap(*global_map,"map");
        last_esft_stamp = ros::Time::now();
        sub_from_local = nh.subscribe<mlmapping::local2global>(
                    "/local2global",
                    10,
                    boost::bind(&LocalMapNodeletClass::from_lm_callback, this, _1));
        
    }

};//class LocalMapNodeletClass
}//namespace mlmapping_ns

PLUGINLIB_EXPORT_CLASS(mlmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)


