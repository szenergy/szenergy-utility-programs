//SZEnergy HUD (prototype/bruteforce/ugly version)

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <autoware_msgs/ControlCommand.h>

const float rtod = 180/M_PI;  //rad-to-deg

//global variables (todo: dynamic recfg)
float freq = 0.2;               //refresh frequency
float maxtime = 0.5;            //max time to wait for topics

std_msgs::Float32 a_temp, s_temp;

jsk_rviz_plugins::OverlayText stxt;     //status text
jsk_rviz_plugins::OverlayText ctxt;     //mission/state text

ros::Subscriber sub_refvec;             //ctrlCmd speed and angle reference
ros::Subscriber sub_mtxt;               //mission ID
ros::Subscriber sub_ctxt;               //state machine string
ros::Publisher  pub_ctxt;               //mission/state text
std::string mid = "-1", smt = "No Data";

ros::Publisher pub_stxt;
ros::Publisher pub_a_ref;               //angle reference (ctrlCmd)
ros::Publisher pub_s_ref;               //speed reference (ctrlCmd)

const std::string base_text = "TOPIC STATUS:\n";
const std::string oktext = "<span style=\"color: cyan;\">OK</span>";
const std::string notoktext = "<span style=\"color: red;\">N/A</span>";
std::string status_text;

struct topok
{
    ros::Subscriber s;          //subscriber
    ros::Publisher p;           //publisher
    std::string tn;             //topic name
    //std::string tt;           //topic type (heheh, as if... - to do for next time?)
    std::string name;           //display name
    std::string outmsg;         //output text (if not okonly)
    ros::Time ts;               //time stamp
    bool ok = false;            //status
    bool okonly = true;         //view only status (bool)

    topok(std::string name_in, std::string tn_in)
    {
        name = name_in;
        tn = tn_in;
    }
};

std::vector<topok> status; //text-based topic status indicator ("topic: OK")

//callbacks

//void cb_mtxt    (const std_msgs::UInt32 &data_in)                       {mid =  std::string(data_in.data);  ctxt.text = "mission ID: " + mid + "\n" + smt;}
void cb_mtxt    (const std_msgs::String &data_in)                       {mid =  data_in.data;               ctxt.text = "mission ID: " + mid + "\n" + smt;}
void cb_ctxt    (const std_msgs::String &data_in)                       {smt =  data_in.data;               ctxt.text = "mission ID: " + mid + "\n" + smt;}

void cb_model   (const visualization_msgs::Marker &data_in)             {status[0].ts = ros::Time::now();}
void cb_ouster  (const pcl::PCLPointCloud2ConstPtr &data_in)            {status[1].ts = ros::Time::now();}
void cb_stop    (const geometry_msgs::PoseStamped &data_in)             {status[2].ts = ros::Time::now();}
void cb_park    (const geometry_msgs::PoseStamped &data_in)             {status[3].ts = ros::Time::now();}
void cb_occup   (const nav_msgs::OccupancyGrid &data_in)                {status[4].ts = ros::Time::now();}
void cb_lapoint (const geometry_msgs::PoseStamped &data_in)             {status[5].ts = ros::Time::now();}
void cb_traj	(const visualization_msgs::MarkerArray &data_in)        {status[6].ts = ros::Time::now();}
void cb_angle   (const std_msgs::Float32 &data_in)                      {status[7].ts = ros::Time::now();}
void cb_speed   (const std_msgs::Float32 &data_in)                      {status[8].ts = ros::Time::now();}
void cb_ref     (const autoware_msgs::ControlCommand &data_in)          {status[9].ts = ros::Time::now();
                                                                            a_temp.data = data_in.steering_angle * rtod;
                                                                            pub_a_ref.publish(a_temp);
                                                                            s_temp.data = data_in.linear_velocity;
                                                                            pub_s_ref.publish(s_temp);   }
void cb_gps     (const geometry_msgs::PoseStamped &data_in)             {status[10].ts = ros::Time::now();}
void cb_gps_s   (const std_msgs::String &data_in)                       {status[11].outmsg = "<span style=\"color: yellow;\">" + data_in.data + "</span>";}

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

        status_text += "\n" + status[i].name + "|" + status[i].outmsg;
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

    status.push_back(topok("model", "/szemission_3d/szemission_marker"));
    status.push_back(topok("ouster", "/os_cloud_node/points"));
    status.push_back(topok("stop line", "/stop_line_pose"));
    status.push_back(topok("park-area", "/park_goal_pose"));
    status.push_back(topok("occup map", "/occupancy_map"));
    status.push_back(topok("look fwd p", "/lookAheadPoint"));
    status.push_back(topok("trajectory", "/polynomial_trajectory"));
    status.push_back(topok("steer ang", "/wheel_angle_deg"));
    status.push_back(topok("speed", "/vehicle_speed_kmph"));
    status.push_back(topok("ctrl cmd", "/ctrlCmd"));
    status.push_back(topok("gps", "/current_pose"));
    status.push_back(topok("gps status", "/gps/duro/status_string"));

    //auto-adjsted equal-length names (unfortunately does not work with whitespace characters)
    {
        int l = 0, i, s = status.size();
        for (i=0; i<s; i++) if (l<status[i].name.size()) l = status[i].name.size();
        for (i=0; i<s; i++) status[i].name.resize(l, '_');
    }

    status[ 0].s = (nh.subscribe(status[ 0].tn, 1, cb_model));
    status[ 1].s = (nh.subscribe(status[ 1].tn, 1, cb_ouster));
    status[ 2].s = (nh.subscribe(status[ 2].tn, 1, cb_stop));
    status[ 3].s = (nh.subscribe(status[ 3].tn, 1, cb_park));
    status[ 4].s = (nh.subscribe(status[ 4].tn, 1, cb_occup));
    status[ 5].s = (nh.subscribe(status[ 5].tn, 1, cb_lapoint));
    status[ 6].s = (nh.subscribe(status[ 6].tn, 1, cb_traj));
    status[ 7].s = (nh.subscribe(status[ 7].tn, 1, cb_angle));
    status[ 8].s = (nh.subscribe(status[ 8].tn, 1, cb_speed));
    status[ 9].s = (nh.subscribe(status[ 9].tn, 1, cb_ref));
    status[10].s = (nh.subscribe(status[10].tn, 1, cb_gps));
    status[11].s = (nh.subscribe(status[11].tn, 1, cb_gps_s));

    status[11].okonly = false;

    sub_mtxt = nh.subscribe("/smMissionID", 1, cb_mtxt);
    sub_ctxt = nh.subscribe("/smState", 1, cb_ctxt);

    pub_stxt = nh.advertise<jsk_rviz_plugins::OverlayText>("status_text", 1);           //all-in-one status text publisher
    pub_ctxt = nh.advertise<jsk_rviz_plugins::OverlayText>("mission_state_text", 1);    //challenge state (text) publisher
    pub_a_ref = nh.advertise<std_msgs::Float32>("wheel_angle_deg_ref", 1);              //ctrlCmd angle reference
    pub_s_ref = nh.advertise<std_msgs::Float32>("vehicle_speed_kmph_ref", 1);           //ctrlCmd speed reference

    //init (to show even when no data)
    a_temp.data = 0.0;
    pub_a_ref.publish(a_temp);
    s_temp.data = 0.0;
    pub_s_ref.publish(s_temp); 

    ros::spin();
}
