//SZEnergy HUD (prototype/bruteforce/ugly version)

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <jsk_rviz_plugins/OverlayText.h>

//global variables (todo: dynamic recfg)
float freq = 0.2;               //refresh frequency
float maxtime = 0.5;            //max time to wait for topics

jsk_rviz_plugins::OverlayText stxt;     //status text
jsk_rviz_plugins::OverlayText ctxt;     //challenge state text

ros::Subscriber sub_ctxt;           //challenge_state
ros::Publisher  pub_ctxt;

ros::Publisher pub_stxt;

const std::string base_text = "TOPIC STATUS REPORT:\n";
const std::string oktext = "<span style=\"color: green;\">OK</span>";
const std::string notoktext = "<span style=\"color: red;\">N/A</span>";
std::string status_text;

struct topok
{
    ros::Subscriber s;          //subscriber
    ros::Publisher p;           //publisher
    std::string tn;             //topic name
    //std::string tt;           //topic type (heheh, as if...)
    std::string name;           //display name
    std::string outmsg;         //output text (if not okonly)
    ros::Time ts;               //time stamp
    bool ok = false;            //status
    bool okonly = true;         //view only status (bool)

    topok(std::string tn_in, std::string name_in)
    {
        tn = tn_in;
        name = name_in;
    }
};

std::vector<topok> status; //text-based topic status indicator ("topic: OK")

//callbacks

void cb_ctxt    (const std_msgs::Int8 &data_in)                 {ctxt.text = " " + std::to_string(data_in.data);}

void cb_ouster  (const pcl::PCLPointCloud2ConstPtr &data_in)    {status[0].ts = ros::Time::now();}
void cb_sick    (const sensor_msgs::LaserScan &data_in)         {status[1].ts = ros::Time::now();}
void cb_gps     (const geometry_msgs::PoseStamped &data_in)     {status[2].ts = ros::Time::now();}
void cb_gps_s   (const std_msgs::String &data_in)               {status[3].outmsg = "<span style=\"color: blue;\">" + data_in.data + "</span>";}
void cb_left    (const visualization_msgs::Marker &data_in)     {status[4].ts = ros::Time::now();}
void cb_right   (const visualization_msgs::Marker &data_in)     {status[5].ts = ros::Time::now();}
void cb_traj	(const visualization_msgs::MarkerArray &data_in){status[6].ts = ros::Time::now();}
void cb_angle   (const std_msgs::Float32 &data_in)              {status[7].ts = ros::Time::now();}
void cb_speed   (const std_msgs::Float32 &data_in)              {status[8].ts = ros::Time::now();}

void timerCallback(const ros::TimerEvent& event)
{
    float time;
    stxt.text = base_text;
    status_text = "";
    for (int i = 0; i < status.size(); i++)
    {
        time = ros::Duration(ros::Time::now() - status[i].ts).toSec();
        if (time < maxtime )
        {
            status[i].ok = true;
        }
        else
        {
            status[i].ok = false;
        }
        if (status[i].okonly)
        {
            if (status[i].ok) status[i].outmsg = oktext;
            else status[i].outmsg = notoktext;
        }

        status_text += "\n" + status[i].name + ": " + status[i].outmsg;
        stxt.text = base_text + status_text;
    }
    
    pub_stxt.publish(stxt);
    pub_ctxt.publish(ctxt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SZEmission_HUD");
    ros::NodeHandle nh;
    ROS_INFO("HUD node started.");

    ros::Timer timer = nh.createTimer(ros::Duration(freq), timerCallback);

    status.push_back(topok("/os1/os1_cloud_node/points", "ouster"));
    status.push_back(topok("/scan", "sick"));
    status.push_back(topok("/current_pose", "gps"));
    status.push_back(topok("/gps/duro/status_string", "gps status"));
    status.push_back(topok("/left_lane", "left lane"));
    status.push_back(topok("/right_lane", "right lane"));
    status.push_back(topok("/waypoints_saver_marker", "trajectory"));
    //etc: (needed?)
    status.push_back(topok("/wheel_angle_deg", "steering angle"));
    status.push_back(topok("/vehicle_speed_kmph", "speed"));

    status[0].s = (nh.subscribe(status[0].tn, 1, cb_ouster));
    status[1].s = (nh.subscribe(status[1].tn, 1, cb_sick));
    status[2].s = (nh.subscribe(status[2].tn, 1, cb_gps));
    status[3].s = (nh.subscribe(status[3].tn, 1, cb_gps_s));
    status[4].s = (nh.subscribe(status[4].tn, 1, cb_left));
    status[5].s = (nh.subscribe(status[5].tn, 1, cb_right));
    status[6].s = (nh.subscribe(status[6].tn, 1, cb_traj));
    status[7].s = (nh.subscribe(status[7].tn, 1, cb_angle));
    status[8].s = (nh.subscribe(status[8].tn, 1, cb_speed));

    status[3].okonly = false;

    sub_ctxt = nh.subscribe("/challenge_state", 1, cb_ctxt);

    pub_stxt = nh.advertise<jsk_rviz_plugins::OverlayText>("status_text", 1); //all-in-one status text publisher
    pub_ctxt = nh.advertise<jsk_rviz_plugins::OverlayText>("challenge_state_text", 1); //challenge state (text) publisher
    
    ctxt.text = "0";

    ros::spin();
}
