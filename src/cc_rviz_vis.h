#ifndef CC_RVIZ_VIS_H
#define CC_RVIZ_VIS_H
#include <ros/ros.h>
#include <utils/include/all_utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class cc_rviz_vis
{
public:
    cc_rviz_vis();
    ~cc_rviz_vis();

    ros::Publisher localmap_pub;
    string localmap_frame_id;
    double min_z;
    double max_z;
    double range_z;

    cc_rviz_vis(ros::NodeHandle& nh,
                string topic_name,
                string frame_id,
                unsigned int buffer_size,
                double minz,
                double maxz);

    void pub_localmap(const vector<Vec3>& pts3d,
                      const ros::Time stamp);


};

#endif // CC_RVIZ_VIS_H
