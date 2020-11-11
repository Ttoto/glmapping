#ifndef MSG_LOCALMAP_H
#define MSG_LOCALMAP_H

#include <glmapping/localmap.h>
#include <local_map_cylindrical.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>


class msg_localmap
{
public:
    ros::Publisher localmap_pub;

    msg_localmap();
    msg_localmap(ros::NodeHandle& nh,
                 string topic_name,
                 int buffersize=2);
    void pub(local_map_cylindrical *map,
             ros::Time stamp);
    static void unpack(glmapping::localmapConstPtr msg_ptr,
                local_map_cylindrical *map);
};

#endif // MSG_LOCALMAP_H
