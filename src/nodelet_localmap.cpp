#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <utils/include/all_utils.h>
#include <cc_local_map.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

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
    //flags
    bool publish_tf;
    string frame_id;
    string child_frame_id;
    bool enable_downsample;
    int  downsample_size;
    SE3 T_bs;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        nh.getParam("/ccmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;

        double d_Rho          = getDoubleVariableFromYaml(configFilePath,"ccmapping_d_Rho");
        double d_Phi_deg      = getDoubleVariableFromYaml(configFilePath,"ccmapping_d_Phi_deg");
        double d_Z            = getDoubleVariableFromYaml(configFilePath,"ccmapping_d_Z");
        int    n_Rho          = getIntVariableFromYaml(configFilePath,"ccmapping_n_Rho");
        int    n_Z_below      = getIntVariableFromYaml(configFilePath,"ccmapping_n_Z_below");
        int    n_Z_over       = getIntVariableFromYaml(configFilePath,"ccmapping_n_Z_over");
        Mat4x4  T_bs_mat      = Mat44FromYaml(configFilePath,"T_B_S");
        bool use_exactsync    = getBoolVariableFromYaml(configFilePath,"use_exactsync");
        publish_tf       = getBoolVariableFromYaml(configFilePath,"publish_tf");
        frame_id       = getStringFromYaml(configFilePath,"frame_id");
        child_frame_id = getStringFromYaml(configFilePath,"child_frame_id");
        T_bs = SE3(T_bs_mat.topLeftCorner(3,3),
                   T_bs_mat.topRightCorner(3,1));

        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;
        transformStamped.transform.rotation.w = 1;

        enable_downsample = true;
        downsample_size   = 1000;

        //init map
        local_map = new cc_local_map();
        local_map->setTbs(T_bs);
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
        ros::Rate rate(20.0);
        while(1)
        {
            if(publish_tf)
            {
                transformStamped.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped);
            }
            rate.sleep();
        }


    }

    void pc_pose_input_callback(const sensor_msgs::PointCloud2::ConstPtr & pc_Ptr,
                                const geometry_msgs::PoseStamped::ConstPtr & pose_Ptr)
    {
        tic_toc_ros update_time;
        static int i=0;
        cout << "in the callback " << i++ << endl;

        SE3 T_wb(SO3(Quaterniond(pose_Ptr->pose.orientation.w,
                                 pose_Ptr->pose.orientation.x,
                                 pose_Ptr->pose.orientation.y,
                                 pose_Ptr->pose.orientation.z)),
                 Vec3(pose_Ptr->pose.position.x,
                      pose_Ptr->pose.position.y,
                      pose_Ptr->pose.position.z));
        if(publish_tf)
        {
            SE3 T_ws = T_wb * T_bs;
            transformStamped.header.stamp = pose_Ptr->header.stamp;
            transformStamped.header.frame_id = frame_id;
            transformStamped.child_frame_id = child_frame_id;
            transformStamped.transform.translation.x = T_ws.translation().x();
            transformStamped.transform.translation.y = T_ws.translation().y();
            transformStamped.transform.translation.z = T_ws.translation().z();
            transformStamped.transform.rotation.x = T_ws.so3().unit_quaternion().x();
            transformStamped.transform.rotation.y = T_ws.so3().unit_quaternion().y();
            transformStamped.transform.rotation.z = T_ws.so3().unit_quaternion().z();
            transformStamped.transform.rotation.w = T_ws.so3().unit_quaternion().w();
            br.sendTransform(transformStamped);
        }

        PointCloudP_ptr cloud (new PointCloudP);
        pcl::fromROSMsg (*pc_Ptr, *cloud);
        if(pc_Ptr->is_dense)
        {
        }else
        {//remove invalid pts
            vector<int> index;
            pcl::removeNaNFromPointCloud(*cloud,*cloud,index);
        }
        int pcsize = static_cast<int>(cloud->size());
        vector<Vec3> pc_eigen;
        for(int i=1; i<1000; i++)
        {
            size_t rand_idx = static_cast<size_t>(rand() % pcsize);
            PointP pt = cloud->at(rand_idx);
            pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                    static_cast<double>(pt.y),
                                    static_cast<double>(pt.z)));
        }
        local_map->input_pc_pose(pc_eigen,T_wb);
        update_time.toc("update time");
        //local_map
    }


};//class LocalMapNodeletClass
}//namespace ccmapping_ns

PLUGINLIB_EXPORT_CLASS(ccmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)


