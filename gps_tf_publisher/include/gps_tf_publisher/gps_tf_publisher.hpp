/*
 * gps_tf_publisher.hpp
 *
 *  Created on: Feb 3, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_GPS_TF_PUBLISHER_GPS_TF_PUBLISHER_HPP_
#define INCLUDE_GPS_TF_PUBLISHER_GPS_TF_PUBLISHER_HPP_

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
public:
    AbstractGpsTfPublisher(std::shared_ptr<ros::NodeHandle> nh,
    		const std::string msg_frame_id="map"): nh(nh),
			msg_frame_id(msg_frame_id),
			internal_state(NavMsgTfPublisherState::START)
	{
    	msg_pose.pose.orientation.w = 1.0;
    	msg_pose.header.frame_id = msg_frame_id;
	}

    bool init()
    {
    	pub_current_pose = nh->advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
		ROS_INFO("ROS client successfully initialized");
		return true;
    }
};

}

#endif /* INCLUDE_GPS_TF_PUBLISHER_GPS_TF_PUBLISHER_HPP_ */
