#ifndef CC_RVIZ_VIS_H
#define RVIZ_VIS_H
#include <ros/ros.h>
#include <map_awareness.h>
#include <map_local.h>
#include <utils/include/all_utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class rviz_vis
{
public:
    rviz_vis();
    ~rviz_vis();

    ros::Publisher map_pub;
    string localmap_frame_id;
    string global_frame_id;
    double min_z;
    double max_z;
    double range_z;
    double cube_size_xy;
    double cube_size_z;

    //for localmap
    void set_as_localmap_publisher(ros::NodeHandle& nh,
                string topic_name,
                string frame_id,
                unsigned int buffer_size,
                double minz,
                double maxz);

    //for globalmap
    void set_as_globalmap_publisher(ros::NodeHandle& nh,
                string topic_name,
                string frame_id,
                unsigned int buffer_size,
                double minz,
                double maxz,
                double gm_cube_size_xy,
                double gm_cube_size_z);

    void pub_localmap(awareness_map_cylindrical* localmap,
                      const ros::Time stamp);

    void pub_globalmap(const vector<Vec3>& pts3d,
                       const ros::Time stamp);


};

#endif // RVIZ_VIS_H
