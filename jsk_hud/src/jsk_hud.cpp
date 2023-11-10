//SZEnergy HUD (prototype/bruteforce/ugly version)

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
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
float freq = 0.1;                       //refresh frequency             [seconds]
float maxtime = 0.5;                    //max time to wait for topics   [seconds]
//float color_freq = 0.1;                 //color flash frequency         [seconds] -- currently same as freq


std_msgs::ColorRGBA color_bg;
std_msgs::ColorRGBA color_idle;
//std_msgs::ColorRGBA flashcolor[2];

std_msgs::Float32 a_temp, s_temp, gui_angle, gui_speed;

jsk_rviz_plugins::OverlayText ttxt;     //topic error (no data) text
std_msgs::String stxt;                  //mission/state text

ros::Subscriber sub_refvec;             //ctrl_cmd speed and angle reference
ros::Subscriber sub_mtxt;               //mission ID
ros::Subscriber sub_stxt;               //state machine string
ros::Publisher  pub_mstate_txt;         //mission/state text
std::string mid = "-1", smt = "No Data";

ros::Publisher pub_topic_error_txt;     //error in main topics
ros::Publisher pub_a_ref;               //angle reference (ctrl_cmd) [sync]
ros::Publisher pub_s_ref;               //speed reference (ctrl_cmd) [sync]
ros::Publisher pub_a_gui;               //angle republish (same Hz as "sync" group) [sync]
ros::Publisher pub_s_gui;               //speed republish (same Hz as "sync" group) [sync]

bool allok = false;                     //no errors
bool idle = false;                      //no change
bool fc = false;                        //toggle error color (flashing)



std::string color_prefix[] = {"<span style=\"color: yellow;\">", "<span style=\"color: red;\">"};
std::string flashstr[] = {" !!! ", " --- "};

struct topok
{
    ros::Subscriber s;          //subscriber
    ros::Publisher p;           //publisher
    std::string tn;             //topic name
    //std::string tt;           //topic type (heheh, as if... - to do for next time?)
    std::string name;           //display name
    ros::Time ts;               //time stamp
    bool error = false;         //no data arriving

    topok(std::string name_in, std::string tn_in)
    {
        name = name_in;
        tn = tn_in;
    }
};

std::vector<topok> status; //text-based topic status indicator ("topic: OK")

//callbacks

void cb_mtxt        (const std_msgs::UInt32 &data_in)                       {mid =  std::to_string(data_in.data);   stxt.data = "mission ID: " + mid + "\n" + smt;}
void cb_stxt        (const std_msgs::String &data_in)                       {smt =  data_in.data;                   stxt.data = "mission ID: " + mid + "\n" + smt;}

void cb_occup       (const nav_msgs::OccupancyGrid &data_in)                {status[0].ts = ros::Time::now();}
void cb_lapoint     (const visualization_msgs::Marker &data_in)             {status[1].ts = ros::Time::now();}
void cb_traj        (const visualization_msgs::MarkerArray &data_in)        {status[2].ts = ros::Time::now();}
//void cb_can         (const visualization_msgs::MarkerArray &data_in)        {status[3].ts = ros::Time::now();}
void cb_ctrl_cmd    (const autoware_msgs::ControlCommandStamped &data_in)   {status[3].ts = ros::Time::now();
                                                                            a_temp.data = data_in.cmd.steering_angle * rtod;
                                                                            //pub_a_ref.publish(a_temp);
                                                                            s_temp.data = data_in.cmd.linear_velocity;
                                                                            //pub_s_ref.publish(s_temp);
                                                                        }
void cb_imu     (const sensor_msgs::Imu &data_in)                       {status[4].ts = ros::Time::now();}

void cb_angle   (const std_msgs::Float32 &data_in)                      {gui_angle = data_in;}
void cb_speed   (const std_msgs::Float32 &data_in)                      {gui_speed = data_in;}

void timerCallback(const ros::TimerEvent& event)
{
    float time;

    time = ros::Duration(ros::Time::now() - status[0].ts).toSec();
    allok = true;
//    if ( !status[0].ok || (time > maxtime) )    { status[0].ok = false; ntxt.data += notoktext; }
//    else                                        { status[0].ok = true;  otxt.data += oktext;    }
    for (int i = 0; i < status.size(); i++)
    {
        time = ros::Duration(ros::Time::now() - status[i].ts).toSec();
        if (time > maxtime)
        {
            allok = false;
            status[i].error = true;
        }
        else
        {
            status[i].error = false;
        }
    }
    if (!allok)
    {
        ttxt.bg_color = color_bg;               //set background to visible
        //ttxt.fg_color = flashcolor[fc];         //set text color
        fc = !fc;                               //switch color
        ttxt.text = color_prefix[fc] + flashstr[fc] + "NO DATA FROM:" + flashstr[fc] + "\n";
        for (int i=0; i<status.size(); i++)
        {
            if (status[i].error) ttxt.text += flashstr[fc] + status[i].name + flashstr[fc] + "\n";
        }
        idle = false;
    }
    else if (!idle)
    {
        ttxt.text = "";
        ttxt.fg_color = color_idle;
        ttxt.bg_color = color_idle;
        idle = true;
    }

    //publish values at 'freq' frequency [in seconds]
    pub_a_ref.publish(a_temp);
    pub_s_ref.publish(s_temp);
    pub_a_gui.publish(gui_angle);
    pub_s_gui.publish(gui_speed);

    pub_topic_error_txt.publish(ttxt);
    pub_mstate_txt.publish(stxt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SZEmission_HUD");
    ros::NodeHandle nh;
    ROS_INFO("HUD node started.");

    ros::Timer timer = nh.createTimer(ros::Duration(freq), timerCallback);
/*
    color_bg.r = 1.0;
    color_bg.g = 1.0;
    color_bg.b = 0.0;
    color_bg.a = 1.0;
    flashcolor[0] = color_bg;

    color_bg.r = 0.0;
    color_bg.g = 1.0;
    color_bg.b = 1.0;
    color_bg.a = 1.0;
    flashcolor[1] = color_bg;
*/
    color_bg.r = 0.0;
    color_bg.g = 0.0;
    color_bg.b = 0.0;
    color_bg.a = 8.0;

    color_idle.r = 0.0;
    color_idle.g = 0.0;
    color_idle.b = 0.0;
    color_idle.a = 0.0;

    status.push_back(topok("occupancy map",    "/occupancy_map"));
    status.push_back(topok("look ahead point", "/lookAheadPoint"));
    status.push_back(topok("trajectory",       "/polynomial_trajectory"));
    //status.push_back(topok("CAN messages",     "/wheel_angle_deg"));
    status.push_back(topok("control command",  "/ctrl_cmd"));
    status.push_back(topok("IMU data",         "/imu/data"));

    status[0].s = (nh.subscribe(status[0].tn, 1, cb_occup));
    status[1].s = (nh.subscribe(status[1].tn, 1, cb_lapoint));
    status[2].s = (nh.subscribe(status[2].tn, 1, cb_traj));
    //status[3].s = (nh.subscribe(status[3].tn, 1, cb_can));
    status[3].s = (nh.subscribe(status[3].tn, 1, cb_ctrl_cmd));
    status[4].s = (nh.subscribe(status[4].tn, 1, cb_imu));

    ttxt.fg_color = color_idle;
    ttxt.bg_color = color_idle;
    ttxt.text = "";

    //ttxt.fg_color = flashcolor[0];
    ttxt.bg_color = color_bg;
    ttxt.text = "NO DATA FROM TOPIC:\n\n";
    for (int i=0; i<status.size(); i++)
    {
        if (status[i].error) ttxt.text += status[i].name + "\n";
    }

    sub_mtxt = nh.subscribe("/smMissionID", 1, cb_mtxt);
    sub_stxt = nh.subscribe("/smState", 1, cb_stxt);

    pub_topic_error_txt = nh.advertise<jsk_rviz_plugins::OverlayText>("topic_error", 1);     //topic error (no data) publisher
    pub_mstate_txt = nh.advertise<std_msgs::String>("mission_state_text", 1);   //mission/state (text) publisher
    pub_a_ref = nh.advertise<std_msgs::Float32>("wheel_angle_deg_ref", 1);      //ctrl_cmd angle reference
    pub_s_ref = nh.advertise<std_msgs::Float32>("vehicle_speed_kmph_ref", 1);   //ctrl_cmd speed reference
    pub_a_gui = nh.advertise<std_msgs::Float32>("gui_msg_angle", 1);            //angle republished [at fix Hz]
    pub_s_gui = nh.advertise<std_msgs::Float32>("gui_msg_speed", 1);            //speed republished [at fix Hz]

    //init (to show even when no data)
    a_temp.data = 0.0;      pub_a_ref.publish(a_temp);
    s_temp.data = 0.0;      pub_s_ref.publish(s_temp);
    gui_angle.data = 0.0;   pub_a_gui.publish(gui_angle);
    gui_speed.data = 0.0;   pub_s_gui.publish(gui_speed);

    pub_topic_error_txt.publish(ttxt);
    pub_mstate_txt.publish(stxt);

    ros::spin();
}
