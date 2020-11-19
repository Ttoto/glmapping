#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>

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
#include <map_awareness.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_vis.h>
#include <msg_awareness2local.h>
#include <msg_awareness.h>


#include <mlmapping/awareness2local.h>
#include <map_local.h>
#include <msg_awareness2local.h>
#include <rviz_vis.h>
#include <map_warehouse.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

    ros::Timer timer_;
    ros::Subscriber sub_from_local;
    local_map_cartesian* local_map;
    map_warehouse* warehouse;
    rviz_vis*      globalmap_publisher;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped_T_wl;
    //    Global2OccupancyGrid2D *occupancy_grid_publisher;
    //    Global2ESDF *esfd2d_publisher;
    //    Global2ESDF3DPatch *esfd3d_publisher;
    ros::Time last_esft_stamp;
    ros::Publisher map_pub;

    void from_awareness_callback(const mlmapping::awareness2localConstPtr & msg)
    {
        SE3 T_wa;
        vector<Vec3> l2g_hit_a;
        vector<Vec3> l2g_miss_a;
        ros::Time stamp;
        msg_awareness2local::unpack(msg,T_wa,l2g_hit_a,l2g_miss_a,stamp);
        local_map->input_pc_pose(l2g_hit_a,l2g_miss_a,T_wa,warehouse);
        SE3 T_wl = local_map->T_wl;
        transformStamped_T_wl.transform.translation.x = T_wl.translation().x();
        transformStamped_T_wl.transform.translation.y = T_wl.translation().y();
        transformStamped_T_wl.transform.translation.z = T_wl.translation().z();
        transformStamped_T_wl.header.stamp = stamp;
        br.sendTransform(transformStamped_T_wl);
        globalmap_publisher->pub_local_map(local_map,stamp);
        //        occupancy_grid_publisher->pub_occupancy_grid_2D_from_globalmap(*local_map,stamp);
        //        esfd3d_publisher->pub_ESDF_3D_from_globalmap(*local_map,stamp);
        //        if((ros::Time::now().toSec()-last_esft_stamp.toSec())>0.19)
        //        {
        //            esfd2d_publisher->pub_ESDF_2D_from_globalmap(*local_map,stamp);
        //        }

        if(1)//visulize raycasting
        {
            visualization_msgs::Marker spheres;
            spheres.header.frame_id  = "awareness_frame";
            spheres.header.stamp = stamp;
            spheres.ns = "points";
            spheres.type = visualization_msgs::Marker::SPHERE_LIST;
            spheres.action = visualization_msgs::Marker::ADD;
            spheres.pose.orientation.w =  1.0;
            spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.1;
            spheres.id = 0;
            for (auto pt:l2g_miss_a) {
                geometry_msgs::Point point;
                point.x = pt.x();
                point.y = pt.y();
                point.z = pt.z();
                spheres.points.push_back(point);
                std_msgs::ColorRGBA color;
                color.r= static_cast<float>(1.0);
                color.g= static_cast<float>(0.0);
                color.b= static_cast<float>(0.0);
                color.a= static_cast<float>(1.0);
                spheres.colors.push_back(color);
            }
            if(spheres.points.size()!=0)
            {
                this->map_pub.publish(spheres);
            }
        }
    }

    void timerCb(const ros::TimerEvent& event){
        transformStamped_T_wl.header.stamp = ros::Time::now();
        br.sendTransform(transformStamped_T_wl);
    }

    virtual void onInit()
    {
        cout << "globalmapnode:" << endl;
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        nh.getParam("/mlmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;

        local_map = new local_map_cartesian();
        local_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_d_xyz"),
                            static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_lm_n_xy")),
                            static_cast<unsigned int>(getIntVariableFromYaml(configFilePath,"mlmapping_lm_n_z")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_log_odds_min")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_log_odds_max")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_measurement_hit")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_measurement_miss")),
                            static_cast<float>(getDoubleVariableFromYaml(configFilePath,"mlmapping_lm_occupied_sh")));
        local_map->allocate_memory_for_local_map();
        warehouse = new map_warehouse();

        transformStamped_T_wl.header.frame_id = getStringFromYaml(configFilePath,"world_frame_id");
        transformStamped_T_wl.child_frame_id  = getStringFromYaml(configFilePath,"local_frame_id");
        transformStamped_T_wl.header.stamp=ros::Time::now();
        transformStamped_T_wl.transform.translation.x = 0;
        transformStamped_T_wl.transform.translation.y = 0;
        transformStamped_T_wl.transform.translation.z = 0;
        transformStamped_T_wl.transform.rotation.x = 0;
        transformStamped_T_wl.transform.rotation.y = 0;
        transformStamped_T_wl.transform.rotation.z = 0;
        transformStamped_T_wl.transform.rotation.w = 1;
        globalmap_publisher =  new rviz_vis();
        globalmap_publisher->set_as_local_map_publisher(nh,"/local_map",
                                                        getStringFromYaml(configFilePath,"local_frame_id"),
                                                        5,
                                                        local_map);
        last_esft_stamp = ros::Time::now();
        sub_from_local = nh.subscribe<mlmapping::awareness2local>(
                    "/awareness2local",
                    2,
                    boost::bind(&LocalMapNodeletClass::from_awareness_callback, this, _1));

        timer_ = nh.createTimer(ros::Duration(0.2), boost::bind(&LocalMapNodeletClass::timerCb, this, _1));
        map_pub = nh.advertise<visualization_msgs::Marker>("raycasting", 3);
    }

};//class LocalMapNodeletClass
}//namespace mlmapping_ns

PLUGINLIB_EXPORT_CLASS(mlmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)

