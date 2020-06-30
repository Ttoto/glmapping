#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>



namespace ccmapping_ns
{

class GlobalMapNodeletClass : public nodelet::Nodelet
{
public:
    GlobalMapNodeletClass()  {;}
    ~GlobalMapNodeletClass() {;}
private:


    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
    }

};//class GlobalMapNodeletClass
}//namespace ccmapping_ns

PLUGINLIB_EXPORT_CLASS(ccmapping_ns::GlobalMapNodeletClass, nodelet::Nodelet)


