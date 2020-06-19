#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>



namespace ccmapping_ns
{

class LocalMapNodeletClass : public nodelet::Nodelet
{
public:
    LocalMapNodeletClass()  {;}
    ~LocalMapNodeletClass() {;}
private:


    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
    }

};//class LocalMapNodeletClass
}//namespace ccmapping_ns

PLUGINLIB_EXPORT_CLASS(ccmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)


