#include <gps_tf_publisher/gps_tf_publisher.hpp>



class NavMsgTfNdtPublisher: public szenergy::AbstractGpsTfPublisher
{
private:
	// Constants
	const std::string msg_frame_id = "world";
	// Var
	// EN: TF world to local map
	// HU: Világból a térkép lokális referencia pontjába

public:
	NavMsgTfNdtPublisher(std::shared_ptr<ros::NodeHandle> nh,
		  const std::string msg_frame_id="map"):
		  szenergy::AbstractGpsTfPublisher(nh, msg_frame_id)
	{

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

	virtual void init() override
	{
		initRosPublishers();
		sub_currentpose = nh->subscribe("/gps/current_pose", 10,
		  &NavMsgTfNdtPublisher::CbCurrentPose, this);
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "leaf_nav_msg_tf_pub");
	std::shared_ptr<ros::NodeHandle> nh =
	std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
	NavMsgTfNdtPublisher nav_msg_tf(nh);
	nav_msg_tf.init();
	ros::spin();
	return 0;
}
