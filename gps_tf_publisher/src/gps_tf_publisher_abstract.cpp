/*
 * gps_tf_publisher_abstract.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: kyberszittya
 */

#include <gps_tf_publisher/gps_tf_publisher.hpp>

namespace szenergy
{
void AbstractGpsTfPublisher::setRelativeReferenceFrame(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// Set reference TF map
	ref_pose.header = msg->header;
	ref_pose.pose = msg->pose;
	tf_world_map.header.frame_id = msg_frame_id;
	tf_world_map.child_frame_id = "map";
	tf_world_map.transform.translation.x = ref_pose.pose.position.x;
	tf_world_map.transform.translation.y = ref_pose.pose.position.y;
	tf_world_map.transform.translation.z = ref_pose.pose.position.z;
	tf2::Quaternion q;
	tf_world_map.transform.rotation.x = ref_pose.pose.orientation.x;
	tf_world_map.transform.rotation.y = ref_pose.pose.orientation.y;
	tf_world_map.transform.rotation.z = ref_pose.pose.orientation.z;
	tf_world_map.transform.rotation.w = ref_pose.pose.orientation.w;
}

void AbstractGpsTfPublisher::updateInstantaneousTransform(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// Propagate stamped messages
	tf_world_map.header.stamp = msg->header.stamp;
	tf_map_baselink.header.stamp = msg->header.stamp;
	tf_map_gps.header.stamp = msg->header.stamp;
	// Set parent frame to the input parameter (e.g. "map", "world")
	tf_map_gps.header.frame_id = msg_frame_id;
	// The child parent frame is always "gps"
	// PROTIP: if you really want to connect GNSS to your main point of reference
	// (e.g. base_link) broadcast a static transform from your launch file
	tf_map_gps.child_frame_id = "gps";
	tf_map_gps.transform.translation.x = msg->pose.position.x;
	tf_map_gps.transform.translation.y = msg->pose.position.y;
	tf_map_gps.transform.translation.z = msg->pose.position.z;
	tf_map_gps.transform.rotation.x = msg->pose.orientation.x;
	tf_map_gps.transform.rotation.y = msg->pose.orientation.y;
	tf_map_gps.transform.rotation.z = msg->pose.orientation.z;
	tf_map_gps.transform.rotation.w = msg->pose.orientation.w;
}

void AbstractGpsTfPublisher::updateInstantaneousRelativePose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// Set reference TF base_link
	// Publish current pose to localization
	msg_pose.header.stamp = msg->header.stamp;
	msg_pose.pose.position.x = msg->pose.position.x;
	msg_pose.pose.position.y = msg->pose.position.y;
	msg_pose.pose.position.z = msg->pose.position.z;
	msg_pose.pose.orientation.x = msg->pose.orientation.x;
	msg_pose.pose.orientation.y = msg->pose.orientation.y;
	msg_pose.pose.orientation.z = msg->pose.orientation.z;
	msg_pose.pose.orientation.w = msg->pose.orientation.w;
}

AbstractGpsTfPublisher::AbstractGpsTfPublisher(
		std::shared_ptr<ros::NodeHandle> nh,
		const std::string msg_frame_id): nh(nh),
		msg_frame_id(msg_frame_id),
		internal_state(NavMsgTfPublisherState::START)
{
	msg_pose.pose.orientation.w = 1.0;
	msg_pose.header.frame_id = msg_frame_id;
}

}
