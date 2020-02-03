
#include <gps_tf_publisher/gps_tf_publisher.hpp>

class NavMsgTfPublisher: public szenergy::AbstractGpsTfPublisher
{
private:

public:
	NavMsgTfPublisher(std::shared_ptr<ros::NodeHandle> nh,
		  const std::string msg_frame_id="map"):
			  szenergy::AbstractGpsTfPublisher(nh, msg_frame_id)
	{
	}

	virtual void init() override
	{
		initRosPublishers();
		sub_currentpose = nh->subscribe("/gps/current_pose", 10,
		  &NavMsgTfPublisher::CbCurrentPose, this);
	}


	void CbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		static tf2_ros::TransformBroadcaster br;
		// We can expect, that current pose will be available
		using namespace szenergy;
		switch(internal_state)
		{
			case NavMsgTfPublisherState::START:
			{
				setRelativeReferenceFrame(msg);
				internal_state = NavMsgTfPublisherState::INITIALIZED;
				ROS_INFO("Initialized TF publisher from first reference message");
				break;
			}
			case NavMsgTfPublisherState::INITIALIZED:
			{
				updateInstantaneousRelativePose(msg);
				updateInstantaneousTransform(msg);
				pub_current_pose.publish(msg_pose);
				// Publish transforms
				br.sendTransform(tf_map_gps);
				break;
			}
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "leaf_nav_msg_tf_pub");

	ros::NodeHandle nh_private("~");
	std::string frame_id("map");
	if (!nh_private.getParam("parent_frame", frame_id))
	{
		ROS_WARN("Setting default frame_id: %s", frame_id.c_str());
	}
	else
	{
		ROS_INFO("Setting frame_id: %s", frame_id.c_str());
	}
	std::shared_ptr<ros::NodeHandle> nh =
			std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
	NavMsgTfPublisher nav_msg_tf(nh, frame_id);
	nav_msg_tf.init();
	ros::spin();
	return 0;
}
