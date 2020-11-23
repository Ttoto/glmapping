#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map_awareness.h>
#include <mlmapping/awareness.h>
#include <msg_awareness.h>
#include <rviz_vis.h>

awareness_map_cylindrical* awareness_map;
local_map_cartesian*       local_map;
rviz_vis*              awareness_map_rviz_pub;
rviz_vis*              local_map_rviz_pub;

void localmap_msg_callback(const mlmapping::awarenessConstPtr awareness_map_msg)
{
  msg_awareness::unpack(awareness_map_msg,awareness_map);
  awareness_map_rviz_pub->pub_awareness_map(awareness_map,awareness_map_msg->header.stamp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visulization node");
  ros::NodeHandle nh;
  string configFilePath;
  nh.getParam("/mlmapping_configfile",   configFilePath);

  //init map and publisher
  //awareness_map
  awareness_map = new awareness_map_cylindrical();
  awareness_map->init_map(getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Rho"),
                          getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Phi_deg"),
                          getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Z"),
                          getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Rho"),
                          getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Z_below"),
                          getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Z_over"),false);
  awareness_map->map_tmp.release();
  awareness_map_rviz_pub =  new rviz_vis();
  awareness_map_rviz_pub->set_as_awareness_map_publisher(nh,"/awareness_map",getStringFromYaml(configFilePath,"awareness_frame_id"),3,awareness_map);
  ros::Subscriber sub = nh.subscribe("/mlmapping_awareness", 1, localmap_msg_callback);
  //local_map
  local_map = new local_map_cartesian();
  local_map_rviz_pub = new rviz_vis();
//  void rviz_vis::set_as_local_map_publisher(ros::NodeHandle& nh,
//                     string topic_name,
//                     string frame_id,
//                     unsigned int buffer_size,
//                     local_map_cartesian* localmap
//  local_map_rviz_pub->set_as_awareness_map_publisher(nh,"/awareness_map",local_frame_id,3,-n_Z_below*d_Z,n_Z_over*d_Z);
//  ros::Subscriber sub = nh.subscribe("/mlmapping_awareness", 1, localmap_msg_callback);
  ros::spin();

  return 0;
}


