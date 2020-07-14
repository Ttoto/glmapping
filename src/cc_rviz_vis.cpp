#include "cc_rviz_vis.h"
#include "cc_local_map.h"

cc_rviz_vis::cc_rviz_vis()
{

}
cc_rviz_vis::~cc_rviz_vis()
{

}

cc_rviz_vis::cc_rviz_vis(ros::NodeHandle& nh,
                         string topic_name,
                         string frame_id,
                         unsigned int buffer_size,
                         double minz,
                         double maxz)
{
    this->localmap_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffer_size);
    this->localmap_frame_id = frame_id;
    this->min_z = minz;
    this->max_z = maxz;
    this->range_z = max_z-min_z;
}

//input: ratio is between 0 to 1
//output: rgb color
Vec3 sphereColer(double ratio)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 5);
    //find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch(normalized / 256)
    {
    case 0: red = 255;      grn = x;        blu = 0;       break;//red
    case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
    case 2: red = 0;        grn = 255;      blu = x;       break;//green
    case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
    case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    }

    return Vec3(red/260.0,grn/260.0,blu/260.0);
}

void cc_rviz_vis::pub_localmap(const vector<Vec3> &pts3d, const ros::Time stamp)
{
    visualization_msgs::Marker spheres;
    spheres.header.frame_id  = this->localmap_frame_id;
    spheres.header.stamp = stamp;
    spheres.ns = "points";
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w =  1.0;
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.2;
    spheres.id = 0;
    spheres.color.a=0.3;
    spheres.color.g=1.0;
    spheres.color.b=0;
    spheres.color.r=0;
    for (auto pt:pts3d) {
        geometry_msgs::Point point;
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        spheres.points.push_back(point);
        double ratio = (pt.z()-min_z)/range_z;
        Vec3 rgb = sphereColer(ratio);
        std_msgs::ColorRGBA color;
        color.r= static_cast<float>(rgb(0));
        color.g= static_cast<float>(rgb(1));
        color.b= static_cast<float>(rgb(2));
        color.a=0.5;
        spheres.colors.push_back(color);
    }



    this->localmap_pub.publish(spheres);
}
