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
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/VehicleStatus.h>

const float rtod = 180/M_PI;  //rad-to-deg

//global variables (todo: dynamic recfg)
float freq = 0.2;                       //refresh frequency             [seconds]
float maxtime = 0.5;                    //max time to wait for topics   [seconds]

std_msgs::Float32 a_temp, s_temp;

std_msgs::String ttxt;                  //topic (sub)list text
std_msgs::String stxt;                  //mission/state text
std_msgs::String otxt;                  //'ok' text column
std_msgs::String ntxt;                  //'not ok' text column

ros::Subscriber sub_refvec;             //ctrl_cmd speed and angle reference
ros::Subscriber sub_mtxt;               //mission ID
ros::Subscriber sub_stxt;               //state machine string
ros::Publisher  pub_stxt;               //mission/state text
std::string mid = "-1", smt = "No Data";

ros::Publisher pub_ttxt;                //topic (sub)list
ros::Publisher pub_otxt;                //formatted text for list of 'ok' topics
ros::Publisher pub_ntxt;                //formatted text for list of 'not ok' topics
ros::Publisher pub_a_ref;               //angle reference (ctrl_cmd)
ros::Publisher pub_s_ref;               //speed reference (ctrl_cmd)

const std::string oktext = "OK";
const std::string notoktext = "X";

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

    topok(std::string name_in, std::string tn_in)
    {
        name = name_in;
        tn = tn_in;
    }
};

std::vector<topok> status; //text-based topic status indicator ("topic: OK")

//callbacks

void cb_mtxt    (const std_msgs::UInt32 &data_in)                       {mid =  std::to_string(data_in.data);   stxt.data = "mission ID: " + mid + "\n" + smt;}
//void cb_mtxt    (const std_msgs::String &data_in)                       {mid =  data_in.data;               stxt.data = "mission ID: " + mid + "\n" + smt;}
void cb_stxt    (const std_msgs::String &data_in)                       {smt =  data_in.data;                   stxt.data = "mission ID: " + mid + "\n" + smt;}

void cb_mode    (const autoware_msgs::VehicleStatus &data_in)           {if (data_in.drivemode)  status[0].ok = true;
                                                                         else                    status[0].ok = false;
                                                                         status[0].ts = ros::Time::now();}
void cb_stop    (const geometry_msgs::PoseStamped &data_in)             {status[1].ts = ros::Time::now();}
void cb_park    (const geometry_msgs::PoseStamped &data_in)             {status[2].ts = ros::Time::now();}
void cb_occup   (const nav_msgs::OccupancyGrid &data_in)                {status[3].ts = ros::Time::now();}
void cb_lapoint (const visualization_msgs::Marker &data_in)             {status[4].ts = ros::Time::now();}
void cb_traj	(const visualization_msgs::MarkerArray &data_in)        {status[5].ts = ros::Time::now();}
void cb_angle   (const std_msgs::Float32 &data_in)                      {status[6].ts = ros::Time::now();}
void cb_speed   (const std_msgs::Float32 &data_in)                      {status[7].ts = ros::Time::now();}
void cb_ref     (const autoware_msgs::ControlCommandStamped &data_in)   {status[8].ts = ros::Time::now();
                                                                            a_temp.data = data_in.cmd.steering_angle * rtod;
                                                                            pub_a_ref.publish(a_temp);
                                                                            s_temp.data = data_in.cmd.linear_velocity;
                                                                            pub_s_ref.publish(s_temp);   }
void cb_gps     (const geometry_msgs::PoseStamped &data_in)             {status[9].ts = ros::Time::now();}

void timerCallback(const ros::TimerEvent& event)
{
    float time;
    otxt.data = "";
    ntxt.data = "";

    time = ros::Duration(ros::Time::now() - status[0].ts).toSec();
    if ( !status[0].ok || (time > maxtime) )    { status[0].ok = false; ntxt.data += notoktext; }
    else                                        { status[0].ok = true;  otxt.data += oktext;    }
    otxt.data += "\n";
    ntxt.data += "\n";

    for (int i = 1; i < status.size(); i++)
    {

        time = ros::Duration(ros::Time::now() - status[i].ts).toSec();
        if (time <= maxtime )
        {
            status[i].ok = true;
            otxt.data += oktext;
        }
        else
        {
            status[i].ok = false;
            ntxt.data += notoktext;
        }
        otxt.data += "\n";
        ntxt.data += "\n";
    }

    pub_ttxt.publish(ttxt);
    pub_otxt.publish(otxt);
    pub_ntxt.publish(ntxt);
    pub_stxt.publish(stxt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SZEmission_HUD");
    ros::NodeHandle nh;
    ROS_INFO("HUD node started.");

    ros::Timer timer = nh.createTimer(ros::Duration(freq), timerCallback);

    status.push_back(topok("autonom mode", "/vehicle_status"));
    status.push_back(topok("stop l. pose", "/stop_line_pose"));
    status.push_back(topok("parking pose", "/park_goal_pose"));
    status.push_back(topok("occup map", "/occupancy_map"));
    status.push_back(topok("look fwd p", "/lookAheadPoint"));
    status.push_back(topok("trajectory", "/polynomial_trajectory"));
    status.push_back(topok("steer angle", "/wheel_angle_deg"));
    status.push_back(topok("veh. speed", "/vehicle_speed_kmph"));
    status.push_back(topok("ctrl cmd", "/ctrl_cmd"));
    status.push_back(topok("gps ↴", "/current_pose"));

    status[0].s = (nh.subscribe(status[0].tn, 1, cb_mode));
    status[1].s = (nh.subscribe(status[1].tn, 1, cb_stop));
    status[2].s = (nh.subscribe(status[2].tn, 1, cb_park));
    status[3].s = (nh.subscribe(status[3].tn, 1, cb_occup));
    status[4].s = (nh.subscribe(status[4].tn, 1, cb_lapoint));
    status[5].s = (nh.subscribe(status[5].tn, 1, cb_traj));
    status[6].s = (nh.subscribe(status[6].tn, 1, cb_angle));
    status[7].s = (nh.subscribe(status[7].tn, 1, cb_speed));
    status[8].s = (nh.subscribe(status[8].tn, 1, cb_ref));
    status[9].s = (nh.subscribe(status[9].tn, 1, cb_gps));

    ttxt.data = "TOPIC STATUS:\n\n";
    for (int i=0; i<status.size(); i++) ttxt.data += status[i].name + "\n";

    sub_mtxt = nh.subscribe("/smMissionID", 1, cb_mtxt);
    sub_stxt = nh.subscribe("/smState", 1, cb_stxt);

    pub_ttxt = nh.advertise<std_msgs::String>("status_topic_text", 1);          //topic (sub)list publisher
    pub_otxt = nh.advertise<std_msgs::String>("status_ok_text", 1);             //status 'ok' text publisher
    pub_ntxt = nh.advertise<std_msgs::String>("status_notok_text", 1);          //status 'not ok' text publisher
    pub_stxt = nh.advertise<std_msgs::String>("mission_state_text", 1);         //mission/state (text) publisher
    pub_a_ref = nh.advertise<std_msgs::Float32>("wheel_angle_deg_ref", 1);      //ctrl_cmd angle reference
    pub_s_ref = nh.advertise<std_msgs::Float32>("vehicle_speed_kmph_ref", 1);   //ctrl_cmd speed reference

    //init (to show even when no data)
    a_temp.data = 0.0;
    pub_a_ref.publish(a_temp);
    s_temp.data = 0.0;
    pub_s_ref.publish(s_temp);
    pub_ttxt.publish(ttxt);
    pub_stxt.publish(stxt);

    ros::spin();
}
