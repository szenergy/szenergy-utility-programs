/*
 * gps_tf_publisher.hpp
 *
 *  Created on: Feb 3, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_GPS_TF_PUBLISHER_GPS_TF_PUBLISHER_HPP_
#define INCLUDE_GPS_TF_PUBLISHER_GPS_TF_PUBLISHER_HPP_

#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


namespace szenergy
{

enum class NavMsgTfPublisherState {START, INITIALIZED, ERROR};

class AbstractGpsTfPublisher
{
private:
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	// Constants
    std::string msg_frame_id;
    // Var
    NavMsgTfPublisherState internal_state;
    ros::Subscriber sub_currentpose;
    // Reference pose for initial TF position
    geometry_msgs::PoseStamped ref_pose;
    // Autoware specific transformation
    geometry_msgs::TransformStamped tf_map_gps;
    geometry_msgs::PoseStamped msg_pose;
    // Republish current pose in the map reference frame
    ros::Publisher pub_current_pose;
    // EN: TF map to base link
    // HU: A térkép referencia pontjából a jármű referencia pontjába
    geometry_msgs::TransformStamped tf_map_baselink;
    // EN: TF world to local map
	// HU: Világból a térkép lokális referencia pontjába
	geometry_msgs::TransformStamped tf_world_map;

    void initRosPublishers()
    {
    	pub_current_pose = nh->advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
		ROS_INFO("ROS client successfully initialized");
    }
public:
    AbstractGpsTfPublisher(std::shared_ptr<ros::NodeHandle> nh,
    		const std::string msg_frame_id="map");


    void updateInstantaneousRelativePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /*
     * @brief: You should use this, if you have a relative frame (e.g. "world"),
     *     map is broadcasted by a localization method (e.g. NDT)
     * @param
     */
    void setRelativeReferenceFrame(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateInstantaneousTransform(const geometry_msgs::PoseStamped::ConstPtr& msg);

    virtual void init() = 0;
};

}

#endif /* INCLUDE_GPS_TF_PUBLISHER_GPS_TF_PUBLISHER_HPP_ */
