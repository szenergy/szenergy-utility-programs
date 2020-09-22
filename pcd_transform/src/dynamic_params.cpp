#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pcd_transform/MyParamsConfig.h>


void callback(pcd_transform::MyParamsConfig &config, uint32_t level){
    
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pcd_transform");

    dynamic_reconfigure::Server<pcd_transform::MyParamsConfig> server;
    dynamic_reconfigure::Server<pcd_transform::MyParamsConfig>::CallbackType f;

    f=boost::bind(&callback,_1,_2);
    server.setCallback(f);

    ros::spin();

    return 0;
}