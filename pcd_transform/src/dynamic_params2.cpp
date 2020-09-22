#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pcd_transform/MyParamsQuaterionConfig.h>


void callback(pcd_transform_quaterion::MyParamsQuaterionConfig &config, uint32_t level){
    
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pcd_transform_quaterion");

    dynamic_reconfigure::Server<pcd_transform_quaterion::MyParamsQuaterionConfig> server;
    dynamic_reconfigure::Server<pcd_transform_quaterion::MyParamsQuaterionConfig>::CallbackType f;

    f=boost::bind(&callback,_1,_2);
    server.setCallback(f);

    ros::spin();

    return 0;
}