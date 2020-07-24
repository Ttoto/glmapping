#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ccmapping/local2global.h>
#include <global_map_cartesian.h>
#include <msg_local2global.h>
#include <rviz_vis.h>

namespace ccmapping_ns
{

class GlobalMapNodeletClass : public nodelet::Nodelet
{
public:
    GlobalMapNodeletClass()  {;}
    ~GlobalMapNodeletClass() {;}
private:

    ros::Subscriber sub_from_local;
    global_map_cartesian* global_map;
    rviz_vis *globalmap_publisher;

    void from_lm_callback(const ccmapping::local2globalConstPtr& msg)
    {
        SE3 T_wl;
        vector<Vec3> l2g_obs_l;
        vector<Vec3> l2g_miss_l;
        ros::Time stamp;
        msg_local2global::unpack(msg,T_wl,l2g_obs_l,l2g_miss_l,stamp);
        cout << "obs pt" << l2g_obs_l.size() << endl;
        global_map->input_pc_pose(l2g_obs_l,T_wl);
        globalmap_publisher->pub_globalmap(global_map->visualization_cell_list,stamp);
    }

    virtual void onInit()
    {
        cout << "globalmapnode:" << endl;
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        cout << "this is ccmapping global node" << endl;
        nh.getParam("/ccmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;
        double d_x  = getDoubleVariableFromYaml(configFilePath,"ccmapping_gm_d_x");
        double d_y  = getDoubleVariableFromYaml(configFilePath,"ccmapping_gm_d_y");
        double d_z  = getDoubleVariableFromYaml(configFilePath,"ccmapping_gm_d_z");
        int    n_x  = getIntVariableFromYaml(configFilePath,"ccmapping_gm_n_x");
        int    n_y  = getIntVariableFromYaml(configFilePath,"ccmapping_gm_n_y");
        int    n_z  = getIntVariableFromYaml(configFilePath,"ccmapping_gm_n_z");
        double min_z = getDoubleVariableFromYaml(configFilePath,"ccmapping_gm_min_z");

        global_map = new global_map_cartesian();
        global_map->init_map(d_x,d_y,d_z,n_x,n_y,n_z,min_z);
        double max_z=min_z+(d_z*n_z);
        globalmap_publisher =  new rviz_vis(nh,"/globalmap","map",2,min_z,max_z,d_x,d_z);
        sub_from_local = nh.subscribe<ccmapping::local2global>(
                    "/local2global",
                    10,
                    boost::bind(&GlobalMapNodeletClass::from_lm_callback, this, _1));


        
    }

};//class GlobalMapNodeletClass
}//namespace ccmapping_ns

PLUGINLIB_EXPORT_CLASS(ccmapping_ns::GlobalMapNodeletClass, nodelet::Nodelet)


