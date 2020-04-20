#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include <cmath>


geometry_msgs::PoseStamped duro_msg;
geometry_msgs::PoseStamped nova_msg;
ros::Publisher pose_pub;

void posenCallback(const geometry_msgs::PoseStamped &msg)
{
    nova_msg = msg;
}

void posedCallback(const geometry_msgs::PoseStamped &msg)
{
    duro_msg = msg;
    auto nx = nova_msg.pose.position.x;
    auto ny = nova_msg.pose.position.y;
    auto dx = duro_msg.pose.position.x;
    auto dy = duro_msg.pose.position.y;
    auto distance = sqrt(pow((nx - dx),2) + pow((ny - dy),2));
    tf2::Quaternion tf_nova(nova_msg.pose.orientation.x, nova_msg.pose.orientation.y, nova_msg.pose.orientation.z, nova_msg.pose.orientation.w);
    tf2::Quaternion tf_duro(duro_msg.pose.orientation.x, duro_msg.pose.orientation.y, duro_msg.pose.orientation.z, duro_msg.pose.orientation.w);
    tf2::Matrix3x3 matn(tf_nova);
    tf2::Matrix3x3 matd(tf_duro);    
    double rolln, pitchn, yawn;
    double rolld, pitchd, yawd;
    matn.getEulerYPR(yawn, pitchn, rolln);
    matd.getEulerYPR(yawd, pitchd, rolld);
    double yawdiff = yawd - yawn;
    //ROS_INFO("yaw-dist: %f", yawn - yawd);
    //ROS_INFO("distace:  %f", distance);
    geometry_msgs::PoseArray pose_msg;
    geometry_msgs::Pose tmppose;
    tmppose.position.x = 0;
    tmppose.position.y = 0;
    pose_msg.poses.push_back(tmppose);
    tmppose.position.x = sin(yawdiff) * distance;
    tmppose.position.y = cos(yawdiff) * distance;
    pose_msg.poses.push_back(tmppose);
    pose_pub.publish(pose_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_compare");
    ros::NodeHandle n;
    pose_pub = n.advertise<geometry_msgs::PoseArray>("/gps/cmparray", 100);
    ros::Subscriber subd = n.subscribe("gps/duro/current_pose", 1000, posedCallback);
    ros::Subscriber subn = n.subscribe("gps/nova/current_pose", 1000, posenCallback);
    ROS_INFO("started");
    ros::spin();
    return 0;
}