#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <novatel_gps_msgs/NovatelVelocity.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <iomanip>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "sys/times.h"
#include "sys/vtimes.h"
#include <sys/sysinfo.h>
#include <cmath>
#include <sstream>
//#include <nvml.h>

class DataProcessor {
  ros::Subscriber vstatus_sub;
  ros::Subscriber dstatus_sub;
  ros::Subscriber nstatus_sub;
  ros::Subscriber stwheel_sub;
  ros::Subscriber vspeed_sub;
  ros::Subscriber rspeed_sub;

public:
  std::string leaf_driver_mode;
  float leaf_speed;
  float ref_speed;
  float st_wheel_angle;
  ros::Time duro_stime;
  ros::Time nova_stime;
  double duro_time;
  double nova_time;

  std::string duro_rtk;
  std::string nova_rtk;

  std::string ouster_left_ok;
  std::string ouster_right_ok;
  std::string velo_left_ok;
  std::string velo_right_ok;
  std::string sick_ok;

  std::string zed_ok;

  DataProcessor(ros::NodeHandle *nh)
      : leaf_driver_mode("\u001b[31mUNDEF\u001b[0m"), leaf_speed(0.0f),
        ref_speed(0.0f), st_wheel_angle(0.0f), duro_time(0.0f),
        nova_time(0.0f) {
    vstatus_sub = nh->subscribe("/vehicle_status", 1,
                                &DataProcessor::vehicleStatusCallback, this);
    dstatus_sub = nh->subscribe("/gps/duro/status_string", 1,
                                &DataProcessor::duroRtkStatusCallBack, this);
    stwheel_sub = nh->subscribe("/wheel_angle_deg", 1,
                                &DataProcessor::steeringWheelCallBack, this);
    vspeed_sub = nh->subscribe("/vehicle_speed_kmph", 1,
                               &DataProcessor::speedCallBack, this);
    rspeed_sub =
        nh->subscribe("/ctrl_cmd", 1, &DataProcessor::refSpeedCallBack, this);

    try {
      nstatus_sub = nh->subscribe("/gps/nova/bestvel", 1,
                                  &DataProcessor::novaRtkStatusCallback, this);
    } catch (const ros::Exception &e) {
      ROS_WARN("no novatel_gps_msgs::msg custom messages built");
    }
    init();
    duro_stime = ros::Time::now();
    nova_stime = ros::Time::now();
  }

  void vehicleStatusCallback(const autoware_msgs::VehicleStatus &msg) {
    if (msg.drivemode == 0)
      leaf_driver_mode = "DRIVER";
    else if (msg.drivemode == 1)
      leaf_driver_mode = "AUTONOMOUS";
    else
      leaf_driver_mode = "\u001b[31mUNDEF\u001b[0m";
  }

  void duroRtkStatusCallBack(const std_msgs::String &msg) {
    if (msg.data != duro_rtk) {
      duro_stime = ros::Time::now();
    }
    duro_time = (ros::Time::now() - duro_stime).toSec();
    duro_time = (duro_time <= 0 ? 0.0f : duro_time);
    duro_rtk = msg.data;
  }

  void novaRtkStatusCallback(const novatel_gps_msgs::NovatelVelocity &msg) {
    if (msg.velocity_type != nova_rtk) {
      nova_stime = ros::Time::now();
    }
    nova_time = (ros::Time::now() - nova_stime).toSec();
    nova_time = (nova_time <= 0 ? 0.0f : nova_time);
    nova_rtk = msg.velocity_type;
  }

  void steeringWheelCallBack(const std_msgs::Float32 &msg) {
    st_wheel_angle = msg.data;
  }

  void speedCallBack(const std_msgs::Float32 &msg) { leaf_speed = msg.data; }

  void refSpeedCallBack(const autoware_msgs::ControlCommandStamped &msg) {
    ref_speed = msg.cmd.linear_velocity;
  }

  void getLeftOusterMsg() {
    ros::Duration timeout(0.23);
    sensor_msgs::PointCloud2ConstPtr ouster_left =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/left_os1/os1_cloud_node/points", timeout);
    if (ouster_left == NULL) {
      ouster_left_ok = "\u001b[31mERR\u001b[0m";
    } else {
      ouster_left_ok = "OK";
    }
  }

  void getRightOusterMsg() {
    ros::Duration timeout(0.23);
    sensor_msgs::PointCloud2ConstPtr ouster_right =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/right_os1/os1_cloud_node/points", timeout);
    if (ouster_right == NULL) {
      ouster_right_ok = "\u001b[31mERR\u001b[0m";
    } else {
      ouster_right_ok = "OK";
    }
  }

  void getLeftVeloMsg() {
    ros::Duration timeout(0.23);
    sensor_msgs::PointCloud2ConstPtr velo_left =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/velodyne_left/velodyne_points", timeout);
    if (velo_left == NULL) {
      velo_left_ok = "\u001b[31mERR\u001b[0m";
    } else {
      velo_left_ok = "OK";
    }
  }

  void getRightVeloMsg() {
    ros::Duration timeout(0.23);
    sensor_msgs::PointCloud2ConstPtr velo_right =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/velodyne_right/velodyne_points", timeout);
    if (velo_right == NULL) {
      velo_right_ok = "\u001b[31mERR\u001b[0m";
    } else {
      velo_right_ok = "OK";
    }
  }

  void getSickMsg() {
    ros::Duration timeout(0.23);
    sensor_msgs::LaserScanConstPtr sick =
        ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", timeout);
    if (sick == NULL) {
      sick_ok = "\u001b[31mERR\u001b[0m";
    } else {
      sick_ok = "OK";
    }
  }

  void getZedMsg() {
    ros::Duration timeout(0.23);
    sensor_msgs::CompressedImageConstPtr zed =
        ros::topic::waitForMessage<sensor_msgs::CompressedImage>(
            "/zed_node/left/image_rect_color/compressed", timeout);
    if (zed == NULL) {
      zed_ok = "\u001b[31mERR\u001b[0m";
    } else {
      zed_ok = "OK";
    }
  }
  // Total cpu_usage
  unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys,
      lastTotalIdle;

  void init() {
    FILE *file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow,
           &lastTotalSys, &lastTotalIdle);
    fclose(file);
  }

  double getCurrentCPUValue() {
    double percent;
    FILE *file;
    unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow,
           &totalSys, &totalIdle);
    fclose(file);

    if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
        totalSys < lastTotalSys || totalIdle < lastTotalIdle) {
      // Overflow detection. Just skip this value.
      percent = -1.0;
    } else {
      total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) +
              (totalSys - lastTotalSys);
      percent = total;
      total += (totalIdle - lastTotalIdle);
      percent /= total;
      percent *= 100;
    }

    lastTotalUser = totalUser;
    lastTotalUserLow = totalUserLow;
    lastTotalSys = totalSys;
    lastTotalIdle = totalIdle;

    if (percent < 0.0)
      percent = 0.0;

    return percent;
  }

  // RAM usage
  unsigned long getRamUsage() {
    struct sysinfo si;
    sysinfo(&si);
    unsigned long result = si.totalram - si.freeram;
    return result;
  }

  unsigned long getProcessCount() {
    struct sysinfo si;
    sysinfo(&si);
    return si.procs;
  }
  // GPU usage
  /*
  double gpu_usage;
  void gpuCallback(){
    NVML nvml;

    NVMLDeviceManager device_manager{nvml};

    const auto devices_begin = device_manager.devices_begin();
    const auto devices_end = device_manager.devices_end();

    for(auto device = devices_begin;device != devices_end;++device){
      const auto& info = (*device).get_info();
      gpu_usage = info.metrics.memory_utilization;
    }
  }
  */
};

class LayoutPrinter {
  DataProcessor processed_data;
  int height;
  int counter;
  int iterator;
  int width;

public:
  LayoutPrinter(ros::NodeHandle *nh) : processed_data(nh) {
    height = 9;
    counter = 0;
    iterator = 0;
    width = 110;
    setHeight();
  }

  void printData(const std::string &s1, const std::string &s2) {
    std::string sep1 = "";
    std::string sep2 = "";
    unsigned int half_width = width / 2;

    for (int i = 0; i < (half_width - s1.length()); ++i)
      sep1 += " ";
    for (int i = 0; i < (half_width - s2.length()); ++i)
      sep2 += " ";

    std::cout << s1 << sep1 << s2 << sep2 << std::endl;
  }
  void printSingleData(const std::string &s) {
    std::string sep = "";
    for (int i = 0; i < (width - s.length()); ++i)
      sep += " ";
    std::cout << s << sep << std::endl;
  }

  std::string printAngleBar(double angle) {
    std::string result = "|";
    int st_angle = 0;
    if (angle < 0.0f) {
      st_angle =
          (static_cast<int>(angle) < -12 ? -12 : static_cast<int>(angle));
    } else {
      st_angle = (static_cast<int>(angle) > 12 ? 12 : static_cast<int>(angle));
    }

    if (angle < 0.0f) {
      for (int i = 1; i < (13 + st_angle); ++i) {
        result += ".";
      }
      for (int i = (13 + st_angle); i < 13; ++i) {
        result += "-";
      }
      result += "o";
      while (result.length() <= 25) {
        result += ".";
      }
    } else if (angle == 0.0f) {
      result += "............o............";
    } else {
      for (int i = 0; i < 12; i++) {
        result += ".";
      }
      result += "o";
      for (int i = 13; i < (13 + st_angle); ++i) {
        result += "-";
      }
      while (result.length() <= 25) {
        result += ".";
      }
    }
    result += "|" + twoDec(angle);

    return result;
  }

  std::string twoDec(double num) {
    std::string result;
    std::stringstream conv;
    conv << std::fixed << std::setprecision(2) << num;
    conv >> result;
    if (result == "-nan")
      result = "0.0";
    return result;
  }

  void printLayout() {
    // printf("\033[2J\033[1;1H");
    if (iterator == 0)
      processed_data.getLeftOusterMsg();
    if (iterator == 8)
      processed_data.getRightOusterMsg();
    if (iterator == 16)
      processed_data.getLeftVeloMsg();
    if (iterator == 24)
      processed_data.getRightVeloMsg();
    if (iterator == 32)
      processed_data.getSickMsg();
    if (iterator == 40) {
      processed_data.getZedMsg();
      iterator = -1;
    }
    ++iterator;
    setCursorPosition();

    std::string duro_msg("Duro: " + processed_data.duro_rtk + " since:" +
                         twoDec(processed_data.duro_time) + " s");
    std::string novatel_msg("Novatel: " + processed_data.nova_rtk + " since:" +
                            twoDec(processed_data.nova_time) + " s");
    std::string left_ouster_msg =
        "OusterLeft: " + processed_data.ouster_left_ok;
    std::string right_ouster_msg =
        "OusterRight: " + processed_data.ouster_right_ok;
    std::string left_velo_msg = "VeloLeft: " + processed_data.velo_left_ok;
    std::string right_velo_msg = "VeloRight: " + processed_data.velo_right_ok;
    std::string zed_msg = "Zed: " + processed_data.zed_ok;
    std::string sick_msg = "SICK: " + processed_data.sick_ok;
    std::string speed_msg = "Speed: " + twoDec(processed_data.leaf_speed) +
                            "km/h ref: " + twoDec(processed_data.ref_speed) +
                            " km/h";
    std::string st_wheel_msg = printAngleBar(processed_data.st_wheel_angle);
    std::string driver_mode = "Driver mode: " + processed_data.leaf_driver_mode;
    std::string cpu_usage =
        "CPU Load: " + twoDec(processed_data.getCurrentCPUValue()) + "%";
    std::string process =
        "Process count: " + std::to_string(processed_data.getProcessCount());
    std::string ram_alloc =
        "RAM allocation:" +
        std::to_string(processed_data.getRamUsage() / (1024 * 1024)) + " MB";
    printData(duro_msg, novatel_msg);
    printData(left_ouster_msg, right_ouster_msg);
    printData(left_velo_msg, right_velo_msg);
    printData(zed_msg, sick_msg);
    printData(speed_msg, st_wheel_msg);
    printSingleData(driver_mode);
    printSingleData(cpu_usage);
    printSingleData(process);
    printSingleData(ram_alloc);
    ++counter;
  }

  void setHeight() {
    for (int i = 0; i < height; ++i)
      std::cout << "\n";
  }

  void setCursorPosition() {
    std::cout << "\u001b[1000D";
    std::string string_to_print = "\u001b[" + std::to_string(height) + "A";
    std::cout << string_to_print;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_diagnostics");

  ros::NodeHandle nh;

  ros::Rate r(20);

  LayoutPrinter lp(&nh);

  ros::AsyncSpinner spinner(0);

  spinner.start();
  while (ros::ok()) {
    lp.printLayout();
    r.sleep();
  }
  ros::waitForShutdown();
}