#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <utils/include/all_utils.h>
#include <cc_local_map.h>


namespace ccmapping_ns
{

using namespace std;

class LocalMapNodeletClass : public nodelet::Nodelet
{
public:
    LocalMapNodeletClass()  {;}
    ~LocalMapNodeletClass() {;}
private:

    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ExactSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> * exactSync_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproxSyncPolicy;
    message_filters::Synchronizer<ApproxSyncPolicy> * approxSync_;

    cc_local_map* local_map;

    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        nh.getParam("/ccmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;

        double d_Rho        = getDoubleVariableFromYaml(configFilePath,"ccmapping_d_Rho");
        double d_Phi_deg    = getDoubleVariableFromYaml(configFilePath,"ccmapping_d_Phi_deg");
        double d_Z          = getDoubleVariableFromYaml(configFilePath,"ccmapping_d_Z");
        int    n_Rho        = getIntVariableFromYaml(configFilePath,"ccmapping_n_Rho");
        int    n_Z_below    = getIntVariableFromYaml(configFilePath,"ccmapping_n_Z_below");
        int    n_Z_over     = getIntVariableFromYaml(configFilePath,"ccmapping_n_Z_over");

        bool use_exactsync  = getBoolVariableFromYaml(configFilePath,"use_exactsync");
        cout << "creat ccmap" << endl;
        local_map = new cc_local_map();
        local_map->init_map(d_Rho,d_Phi_deg,d_Z,n_Rho,n_Z_below,n_Z_over);

        //init the local map

        if(use_exactsync)
        {
            pc_sub.subscribe(nh,   "/ccmapping/pc", 10);
            pose_sub.subscribe(nh, "/ccmapping/pose", 10);
            exactSync_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(5), pc_sub, pose_sub);
            exactSync_->registerCallback(boost::bind(&LocalMapNodeletClass::pc_pose_input_callback, this, _1, _2));
            cout << "ExactSyncPolicy" << endl;
        }
        else
        {
            pc_sub.subscribe(nh,   "/ccmapping/pc", 1);
            pose_sub.subscribe(nh, "/ccmapping/pose", 1);
            approxSync_ = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(10), pc_sub, pose_sub);
            approxSync_->registerCallback(boost::bind(&LocalMapNodeletClass::pc_pose_input_callback, this, _1, _2));
            cout << "ApproxSyncPolicy" << endl;
        }


    }

    void pc_pose_input_callback(const sensor_msgs::PointCloud2::ConstPtr & pc_Ptr,
                                const geometry_msgs::PoseStamped::ConstPtr & pose_Ptr)
    {
        static int i=0;

        cout << "in the callback " << i++ << endl;
    }


};//class LocalMapNodeletClass
}//namespace ccmapping_ns

PLUGINLIB_EXPORT_CLASS(ccmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)


