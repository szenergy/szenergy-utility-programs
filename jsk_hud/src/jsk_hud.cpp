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
#include <sensor_msgs/Image.h>
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

jsk_rviz_plugins::OverlayText ttxt[3];  //topic error (no data) text\
                                        - hardware, software (essential) and object detection poses separately
std_msgs::String stxt;                  //mission/state text

ros::Subscriber sub_refvec;             //ctrl_cmd speed and angle reference
ros::Subscriber sub_mtxt;               //mission ID
ros::Subscriber sub_stxt;               //state machine string
ros::Publisher  pub_mstate_txt;         //mission/state text
std::string mid = "-1", smt = "No Data";

ros::Publisher pub_hw_topic_error_txt;  //error in hardware topics
ros::Publisher pub_sw_topic_error_txt;  //error in software topics (essentials)
ros::Publisher pub_od_topic_error_txt;  //error in object detection / pose topics
ros::Publisher pub_a_ref;               //angle reference (ctrl_cmd) [sync]
ros::Publisher pub_s_ref;               //speed reference (ctrl_cmd) [sync]
ros::Publisher pub_a_gui;               //angle republish (same Hz as "sync" group) [sync]
ros::Publisher pub_s_gui;               //speed republish (same Hz as "sync" group) [sync]

bool status_ok[3] = {true, true, true}; //no errors
bool idle[3] = {true, true, true};      //no change

bool fc = false;                        //error color variable (flashing)

int lim[] = {0, 3, 8, 11};

std::string color_prefix[] =
{
    "<span style=\"color: red;\">",
    "<span style=\"color: yellow;\">",
    "<span style=\"color: cyan;\">"};
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

void cb_lidar       (const sensor_msgs::PointCloud2 &data_in)               {status[0].ts = ros::Time::now();}
void cb_imu         (const sensor_msgs::Imu &data_in)                       {status[1].ts = ros::Time::now();}
void cb_zed         (const sensor_msgs::Image &data_in)                     {status[2].ts = ros::Time::now();}
void cb_mtxt        (const std_msgs::UInt32 &data_in)                       {mid =  std::to_string(data_in.data);   stxt.data = "mission ID: " + mid + "\n" + smt;}
void cb_stxt        (const std_msgs::String &data_in)                       {smt =  data_in.data;                   stxt.data = "mission ID: " + mid + "\n" + smt;}

void cb_occup       (const nav_msgs::OccupancyGrid &data_in)                {status[3].ts = ros::Time::now();}
void cb_lapoint     (const visualization_msgs::Marker &data_in)             {status[4].ts = ros::Time::now();}
void cb_traj        (const visualization_msgs::MarkerArray &data_in)        {status[5].ts = ros::Time::now();}
void cb_ctrl_cmd    (const autoware_msgs::ControlCommandStamped &data_in)   {status[6].ts = ros::Time::now();
                                                                            a_temp.data = data_in.cmd.steering_angle * rtod;
                                                                            //pub_a_ref.publish(a_temp);
                                                                            s_temp.data = data_in.cmd.linear_velocity;
                                                                            //pub_s_ref.publish(s_temp);
                                                                            }

void cb_angle       (const std_msgs::Float32 &data_in)                      {status[7].ts = ros::Time::now();
                                                                             gui_angle = data_in;}
void cb_speed       (const std_msgs::Float32 &data_in)                      {gui_speed = data_in;}
void cb_stop        (const std_msgs::Bool &data_in)                         {if (data_in.data) status[8].ts = ros::Time::now();}
void cb_line        (const geometry_msgs::PoseStamped &data_in)             {status[9].ts = ros::Time::now();}
void cb_park        (const geometry_msgs::PoseStamped &data_in)             {status[10].ts = ros::Time::now();}



void timerCallback(const ros::TimerEvent& event)
{
    float time;

    time = ros::Duration(ros::Time::now() - status[0].ts).toSec();
    fc = !fc;                                       //switch color
    for (int c=0; c<3; c++)
    {
        status_ok[c] = true;
    //    if ( !status[0].ok || (time > maxtime) )    { status[0].ok = false; ntxt.data += notoktext; }
    //    else                                        { status[0].ok = true;  otxt.data += oktext;    }
        for (int i = lim[c]; i < lim[c+1]; i++)
        {
            time = ros::Duration(ros::Time::now() - status[i].ts).toSec();
            if (time > maxtime)
            {
                status_ok[c] = false;
                status[i].error = true;
            }
            else
            {
                status[i].error = false;
            }
        }
        if (c==2) continue;
        if (!status_ok[c])
        {
            ttxt[c].bg_color = color_bg;               //set background to visible
            //ttxt[0].fg_color = flashcolor[fc];         //set text color
            ttxt[c].text = color_prefix[fc] + "### " + "NO DATA FROM:" + " ###" + "\n";
            for (int i=lim[c]; i<lim[c+1]; i++)
            {
                if (status[i].error) ttxt[c].text += flashstr[fc] + status[i].name + flashstr[fc] + "\n";
            }

            idle[c] = false;
        }
        else if (!idle[c])
        {
            ttxt[c].text = "";
            ttxt[c].fg_color = color_idle;
            ttxt[c].bg_color = color_idle;
            idle[c] = true;
        }
    }

    if (!status_ok[2])
        {
            ttxt[2].text = color_prefix[2];
            for (int i=lim[2]; i<lim[3]; i++)
            {
                if (!status[i].error) ttxt[2].text += flashstr[1] + status[i].name + flashstr[1] + "\n";
            }
            ttxt[2].text += color_prefix[0];
            ttxt[2].text += "### NO DATA FROM: ###\n";
            for (int i=lim[2]; i<lim[3]; i++)
            {
                if (status[i].error) ttxt[2].text += flashstr[0] + status[i].name + flashstr[0] + "\n";
            }

            idle[2] = false;
        }
        else if (!idle[2])
        {
            ttxt[2].text = color_prefix[2];
            for (int i=lim[2]; i<lim[3]; i++)
            {
                if (!status[i].error) ttxt[2].text += flashstr[1] + status[i].name + flashstr[1] + "\n";
            }
            idle[2] = true;
        }

    //publish values at 'freq' frequency [in seconds]
    pub_a_ref.publish(a_temp);
    pub_s_ref.publish(s_temp);
    pub_a_gui.publish(gui_angle);
    pub_s_gui.publish(gui_speed);

    pub_hw_topic_error_txt.publish(ttxt[0]);
    pub_sw_topic_error_txt.publish(ttxt[1]);
    pub_od_topic_error_txt.publish(ttxt[2]);
    pub_mstate_txt.publish(stxt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SZEmission_HUD");
    ros::NodeHandle nh;
    ROS_INFO("HUD node started, initializing...");

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
    color_bg.a = 0.8;

    color_idle.r = 0.0;
    color_idle.g = 0.0;
    color_idle.b = 0.0;
    color_idle.a = 0.0;

    status.push_back(topok("lidar pointcloud", "/ouster/points"));
    status.push_back(topok("IMU data",         "/imu/data"));
    status.push_back(topok("camera (L) image", "/zed2i/zed_node/left/image_rect_color"));
    status.push_back(topok("occupancy map",    "/occupancy_map"));
    status.push_back(topok("look ahead point", "/lookAheadPoint"));
    status.push_back(topok("trajectory",       "/polynomial_trajectory"));
    status.push_back(topok("control command",  "/ctrl_cmd"));
    status.push_back(topok("CAN messages",     "/wheel_angle_deg"));
    status.push_back(topok("stop sign exists",   "/stop_sign_exists"));
    status.push_back(topok("stop line pose",   "/stop_line_pose"));
    status.push_back(topok("park goal pose",   "/park_goal_pose"));

    status[0].s = (nh.subscribe(status[0].tn, 1, cb_lidar));
    status[1].s = (nh.subscribe(status[1].tn, 1, cb_imu));
    status[2].s = (nh.subscribe(status[2].tn, 1, cb_zed));
    status[3].s = (nh.subscribe(status[3].tn, 1, cb_occup));
    status[4].s = (nh.subscribe(status[4].tn, 1, cb_lapoint));
    status[5].s = (nh.subscribe(status[5].tn, 1, cb_traj));
    status[6].s = (nh.subscribe(status[6].tn, 1, cb_ctrl_cmd));
    status[7].s = (nh.subscribe(status[7].tn, 1, cb_angle));
    status[8].s = (nh.subscribe(status[8].tn, 1, cb_stop));
    status[9].s = (nh.subscribe(status[9].tn, 1, cb_line));
    status[10].s = (nh.subscribe(status[10].tn, 1, cb_park));

    for (int i=0; i<3; i++)
    {
        ttxt[i].fg_color = color_idle;
        ttxt[i].bg_color = color_idle;
        ttxt[i].text = "";

        //ttxt[0].fg_color = flashcolor[0];
        ttxt[i].bg_color = color_bg;
    }
/*
    for (int i=0; i<status.size(); i++)
    {
        if (status[i].error) ttxt[0].text += status[i].name + "\n";
    }
*/

    sub_mtxt = nh.subscribe("/smMissionID", 1, cb_mtxt);
    sub_stxt = nh.subscribe("/smState", 1, cb_stxt);

    pub_hw_topic_error_txt = nh.advertise<jsk_rviz_plugins::OverlayText>("topic_error_hw", 1);     //hardware topic error (no data) publisher
    pub_sw_topic_error_txt = nh.advertise<jsk_rviz_plugins::OverlayText>("topic_error_sw", 1);     //software topic error (no data) publisher
    pub_od_topic_error_txt = nh.advertise<jsk_rviz_plugins::OverlayText>("topic_error_od", 1);     //obj.det./pose topic error (no data) publisher
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

    pub_hw_topic_error_txt.publish(ttxt[0]);
    pub_sw_topic_error_txt.publish(ttxt[1]);
    pub_od_topic_error_txt.publish(ttxt[2]);
    pub_mstate_txt.publish(stxt);

    ROS_INFO("HUD node is now ready.");

    ros::spin();
}
