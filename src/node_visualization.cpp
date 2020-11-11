#include <ros/ros.h>
#include <std_msgs/String.h>
#include <local_map_cylindrical.h>
#include <glmapping/localmap.h>
#include <msg_localmap.h>
#include <rviz_vis.h>

local_map_cylindrical* local_map;
rviz_vis*              localmap_rviz_pub;
rviz_vis*              globalmap_rviz_pub;

void localmap_msg_callback(const glmapping::localmapConstPtr localmap_msg)
{
  msg_localmap::unpack(localmap_msg,local_map);
  localmap_rviz_pub->pub_localmap(local_map,localmap_msg->header.stamp);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  cout << "this is visualization node" << endl;


  cout << "visualization_node started" << endl;
  string configFilePath;
  nh.getParam("/glmapping_configfile",   configFilePath);
  double d_Rho          = getDoubleVariableFromYaml(configFilePath,"glmapping_lm_d_Rho");
  double d_Phi_deg      = getDoubleVariableFromYaml(configFilePath,"glmapping_lm_d_Phi_deg");
  double d_Z            = getDoubleVariableFromYaml(configFilePath,"glmapping_lm_d_Z");
  int    n_Rho          = getIntVariableFromYaml(configFilePath,"glmapping_lm_n_Rho");
  int    n_Z_below      = getIntVariableFromYaml(configFilePath,"glmapping_lm_n_Z_below");
  int    n_Z_over       = getIntVariableFromYaml(configFilePath,"glmapping_lm_n_Z_over");
  string local_frame_id        = getStringFromYaml(configFilePath,"localmap_frame_id");

  //init map
  local_map = new local_map_cylindrical();
  local_map->init_map(d_Rho,d_Phi_deg,d_Z,n_Rho,n_Z_below,n_Z_over,false);
  local_map->map_tmp.release();

  localmap_rviz_pub =  new rviz_vis();
  localmap_rviz_pub->set_as_localmap_publisher(nh,"/localmap",local_frame_id,3,-n_Z_below*d_Z,n_Z_over*d_Z);
  ros::Subscriber sub = nh.subscribe("/glmapping_localmap", 1, localmap_msg_callback);

  ros::spin();

  return 0;
}


