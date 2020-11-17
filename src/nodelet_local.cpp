#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <mlmapping/awareness2local.h>
#include <map_local.h>
#include <msg_awareness2local.h>
#include <rviz_vis.h>
#include <map_warehouse.h>
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
    local_map_cartesian* local_map;
    map_warehouse* warehouse;
    rviz_vis *globalmap_publisher;
//    Global2OccupancyGrid2D *occupancy_grid_publisher;
//    Global2ESDF *esfd2d_publisher;
//    Global2ESDF3DPatch *esfd3d_publisher;
    ros::Time last_esft_stamp;

    void from_awareness_callback(const mlmapping::awareness2localConstPtr & msg)
    {
        SE3 T_wa;
        vector<Vec3> l2g_hit_a;
        vector<Vec3> l2g_miss_a;
        ros::Time stamp;
        msg_awareness2local::unpack(msg,T_wa,l2g_hit_a,l2g_miss_a,stamp);
        local_map->input_pc_pose(l2g_hit_a,l2g_miss_a,T_wa);
        vector<unsigned int> idxs;
        //warehouse->searchSubMap(Vec3(0,0,0),20,idxs);
        globalmap_publisher->pub_globalmap(local_map->visualization_cell_list,stamp);
//        occupancy_grid_publisher->pub_occupancy_grid_2D_from_globalmap(*local_map,stamp);
//        esfd3d_publisher->pub_ESDF_3D_from_globalmap(*local_map,stamp);
//        if((ros::Time::now().toSec()-last_esft_stamp.toSec())>0.19)
//        {
//            esfd2d_publisher->pub_ESDF_2D_from_globalmap(*local_map,stamp);
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


        local_map = new local_map_cartesian();
        local_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_d_xyz"),
                            static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_gm_n_xy")),
                            static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_gm_n_z")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_log_odds_min")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_log_odds_max")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_measurement_hit")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_measurement_miss")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_gm_occupied_sh")));

//        double max_z=min_z+(d_z*n_z);
        globalmap_publisher =  new rviz_vis();
//        globalmap_publisher->set_as_globalmap_publisher(nh,"/globalmap","map",2,min_z,max_z,d_x,d_z);
//        occupancy_grid_publisher = new Global2OccupancyGrid2D(nh,"/occupancygrid",2);
//        occupancy_grid_publisher->setGlobalMap(*local_map,"map");
//        esfd2d_publisher = new Global2ESDF(nh,"/esfd_map",2);
//        esfd2d_publisher->setGlobalMap(*local_map,"map");
//        esfd3d_publisher = new Global2ESDF3DPatch(nh,"/esfd_batch",2);
//        esfd3d_publisher->setGlobalMap(*local_map,"map");
        last_esft_stamp = ros::Time::now();
        sub_from_local = nh.subscribe<mlmapping::awareness2local>(
                    "/awareness2local",
                    2,
                    boost::bind(&LocalMapNodeletClass::from_awareness_callback, this, _1));
        
    }

};//class LocalMapNodeletClass
}//namespace mlmapping_ns

PLUGINLIB_EXPORT_CLASS(mlmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)


