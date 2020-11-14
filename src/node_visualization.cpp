#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map_awareness.h>
#include <mlmapping/awareness.h>
#include <msg_awareness.h>
#include <rviz_vis.h>

awareness_map_cylindrical* local_map;
rviz_vis*              localmap_rviz_pub;
rviz_vis*              globalmap_rviz_pub;

void localmap_msg_callback(const mlmapping::awarenessConstPtr localmap_msg)
{
  msg_awareness::unpack(localmap_msg,local_map);
  localmap_rviz_pub->pub_localmap(local_map,localmap_msg->header.stamp);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  cout << "this is visualization node" << endl;


  cout << "visualization_node started" << endl;
  string configFilePath;
  nh.getParam("/mlmapping_configfile",   configFilePath);
  double d_Rho          = getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Rho");
  double d_Phi_deg      = getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Phi_deg");
  double d_Z            = getDoubleVariableFromYaml(configFilePath,"mlmapping_am_d_Z");
  int    n_Rho          = getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Rho");
  int    n_Z_below      = getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Z_below");
  int    n_Z_over       = getIntVariableFromYaml(configFilePath,"mlmapping_am_n_Z_over");
  string local_frame_id        = getStringFromYaml(configFilePath,"awareness_frame_id");

  //init map
  local_map = new awareness_map_cylindrical();
  local_map->init_map(d_Rho,d_Phi_deg,d_Z,n_Rho,n_Z_below,n_Z_over,false);
  local_map->map_tmp.release();

  localmap_rviz_pub =  new rviz_vis();
  localmap_rviz_pub->set_as_localmap_publisher(nh,"/awareness_map",local_frame_id,3,-n_Z_below*d_Z,n_Z_over*d_Z);
  ros::Subscriber sub = nh.subscribe("/mlmapping_awareness", 1, localmap_msg_callback);

  ros::spin();

  return 0;
}


