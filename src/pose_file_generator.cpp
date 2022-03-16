/**
 * @Author: Toni Tauler
 * @File: pose_file_generator.cpp
 * @Description: TODO
 * @Date: February 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/buffer.h> 
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h> // from rpy to quaternion

#include <sstream>
#include <iostream>
#include <time.h>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <bits/stdc++.h>
#include <ctime>

//using namespace std;

// STRUCTS
struct Pose {
  double time;
  geometry_msgs::Pose pose;
};

Pose actual_pose_a;
bool pose_a = false;

Pose actual_pose_b;
bool pose_b = false;

Pose actual_pose_c;
bool pose_c = false;

Pose actual_pose_d;
bool pose_d = false;

Pose actual_pose_e;
bool pose_e = false;

// AMCL
Pose actual_amcl_a;
bool amcl_a = false;

Pose actual_amcl_b;
bool amcl_b = false;

Pose actual_amcl_c;
bool amcl_c = false;

Pose actual_amcl_d;
bool amcl_d = false;

Pose actual_amcl_e;
bool amcl_e = false;

// OPTITRACK
Pose actual_optitrack_a;
bool optitrack_a = false;

Pose actual_optitrack_b;
bool optitrack_b = false;

Pose actual_optitrack_c;
bool optitrack_c = false;

Pose actual_optitrack_d;
bool optitrack_d = false;

Pose actual_optitrack_e;
bool optitrack_e = false;

int pose_information = 0;  // 0 == odom(default) / 1 == amcl / 2 == optitrack / 3 == all methods

bool get_odom = true;
bool get_amcl = false;
bool get_optitrack = false;

char* char_arr;
char* char_odom;
char* char_amcl;
char* char_optitrack;

double init_time;

std::string folder_direction = "/home/tonitauler/catkin_ws/src/file_generator/data";
std::string folder_name = "pere";
std::string odom_folder = "odom";
std::string amcl_folder = "amcl";
std::string optitrack_folder = "optitrack";

// FUNCTIONS
double getYaw(Pose pose);
std::string current_date();

void poseACallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_pose_a.time = sec + (nsec / 1000000000) - init_time;

  actual_pose_a.pose.position.x = pose->pose.pose.position.x;
  actual_pose_a.pose.position.y = pose->pose.pose.position.y;
  actual_pose_a.pose.position.z = pose->pose.pose.position.z;

  actual_pose_a.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_pose_a.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_pose_a.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_pose_a.pose.orientation.w = pose->pose.pose.orientation.w;

  pose_a = true;
}

void poseBCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_pose_b.time = sec + (nsec / 1000000000) - init_time;

  actual_pose_b.pose.position.x = pose->pose.pose.position.x;
  actual_pose_b.pose.position.y = pose->pose.pose.position.y;
  actual_pose_b.pose.position.z = pose->pose.pose.position.z;

  actual_pose_b.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_pose_b.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_pose_b.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_pose_b.pose.orientation.w = pose->pose.pose.orientation.w;
  
  pose_b = true;
}

void poseCCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_pose_c.time = sec + (nsec / 1000000000) - init_time;

  actual_pose_c.pose.position.x = pose->pose.pose.position.x;
  actual_pose_c.pose.position.y = pose->pose.pose.position.y;
  actual_pose_c.pose.position.z = pose->pose.pose.position.z;

  actual_pose_c.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_pose_c.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_pose_c.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_pose_c.pose.orientation.w = pose->pose.pose.orientation.w;
  
  pose_c = true;
}

void poseDCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_pose_d.time = sec + (nsec / 1000000000) - init_time;
  
  actual_pose_d.pose.position.x = pose->pose.pose.position.x;
  actual_pose_d.pose.position.y = pose->pose.pose.position.y;
  actual_pose_d.pose.position.z = pose->pose.pose.position.z;

  actual_pose_d.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_pose_d.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_pose_d.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_pose_d.pose.orientation.w = pose->pose.pose.orientation.w;
  
  pose_d = true;
}

void poseECallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_pose_e.time = sec + (nsec / 1000000000) - init_time;

  actual_pose_e.pose.position.x = pose->pose.pose.position.x;
  actual_pose_e.pose.position.y = pose->pose.pose.position.y;
  actual_pose_e.pose.position.z = pose->pose.pose.position.z;

  actual_pose_e.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_pose_e.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_pose_e.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_pose_e.pose.orientation.w = pose->pose.pose.orientation.w;
  
  pose_e = true;
}

void amclACallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_amcl_a.time = sec + (nsec / 1000000000) - init_time;

  actual_amcl_a.pose.position.x = pose->pose.pose.position.x;
  actual_amcl_a.pose.position.y = pose->pose.pose.position.y;
  actual_amcl_a.pose.position.z = pose->pose.pose.position.z;

  actual_amcl_a.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_amcl_a.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_amcl_a.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_amcl_a.pose.orientation.w = pose->pose.pose.orientation.w;
  
  amcl_a = true;
}

void amclBCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_amcl_b.time = sec + (nsec / 1000000000) - init_time;

  actual_amcl_b.pose.position.x = pose->pose.pose.position.x;
  actual_amcl_b.pose.position.y = pose->pose.pose.position.y;
  actual_amcl_b.pose.position.z = pose->pose.pose.position.z;

  actual_amcl_b.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_amcl_b.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_amcl_b.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_amcl_b.pose.orientation.w = pose->pose.pose.orientation.w;
  
  amcl_b = true;
}

void amclCCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_amcl_c.time = sec + (nsec / 1000000000) - init_time;

  actual_amcl_c.pose.position.x = pose->pose.pose.position.x;
  actual_amcl_c.pose.position.y = pose->pose.pose.position.y;
  actual_amcl_c.pose.position.z = pose->pose.pose.position.z;

  actual_amcl_c.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_amcl_c.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_amcl_c.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_amcl_c.pose.orientation.w = pose->pose.pose.orientation.w;
  
  amcl_c = true;
}

void amclDCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_amcl_d.time = sec + (nsec / 1000000000) - init_time;

  actual_amcl_d.pose.position.x = pose->pose.pose.position.x;
  actual_amcl_d.pose.position.y = pose->pose.pose.position.y;
  actual_amcl_d.pose.position.z = pose->pose.pose.position.z;

  actual_amcl_d.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_amcl_d.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_amcl_d.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_amcl_d.pose.orientation.w = pose->pose.pose.orientation.w;
  
  amcl_d = true;
}

void amclECallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_amcl_e.time = sec + (nsec / 1000000000) - init_time;

  actual_amcl_e.pose.position.x = pose->pose.pose.position.x;
  actual_amcl_e.pose.position.y = pose->pose.pose.position.y;
  actual_amcl_e.pose.position.z = pose->pose.pose.position.z;

  actual_amcl_e.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_amcl_e.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_amcl_e.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_amcl_e.pose.orientation.w = pose->pose.pose.orientation.w;
  
  amcl_e = true;
}

void optitrackACallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_optitrack_a.time = sec + (nsec / 1000000000) - init_time;

  actual_optitrack_a.pose.position.x = pose->pose.position.x;
  actual_optitrack_a.pose.position.y = pose->pose.position.y;
  actual_optitrack_a.pose.position.z = pose->pose.position.z;

  actual_optitrack_a.pose.orientation.x = pose->pose.orientation.x;
  actual_optitrack_a.pose.orientation.y = pose->pose.orientation.y;
  actual_optitrack_a.pose.orientation.z = pose->pose.orientation.z;
  actual_optitrack_a.pose.orientation.w = pose->pose.orientation.w;
  
  optitrack_a = true;
}

void optitrackBCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_optitrack_b.time = sec + (nsec / 1000000000) - init_time;

  actual_optitrack_b.pose.position.x = pose->pose.position.x;
  actual_optitrack_b.pose.position.y = pose->pose.position.y;
  actual_optitrack_b.pose.position.z = pose->pose.position.z;

  actual_optitrack_b.pose.orientation.x = pose->pose.orientation.x;
  actual_optitrack_b.pose.orientation.y = pose->pose.orientation.y;
  actual_optitrack_b.pose.orientation.z = pose->pose.orientation.z;
  actual_optitrack_b.pose.orientation.w = pose->pose.orientation.w;
  
  optitrack_b = true;
}

void optitrackCCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_optitrack_c.time = sec + (nsec / 1000000000) - init_time;

  actual_optitrack_c.pose.position.x = pose->pose.position.x;
  actual_optitrack_c.pose.position.y = pose->pose.position.y;
  actual_optitrack_c.pose.position.z = pose->pose.position.z;

  actual_optitrack_c.pose.orientation.x = pose->pose.orientation.x;
  actual_optitrack_c.pose.orientation.y = pose->pose.orientation.y;
  actual_optitrack_c.pose.orientation.z = pose->pose.orientation.z;
  actual_optitrack_c.pose.orientation.w = pose->pose.orientation.w;
  
  optitrack_c = true;
}

void optitrackDCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_optitrack_d.time = sec + (nsec / 1000000000) - init_time;

  actual_optitrack_d.pose.position.x = pose->pose.position.x;
  actual_optitrack_d.pose.position.y = pose->pose.position.y;
  actual_optitrack_d.pose.position.z = pose->pose.position.z;

  actual_optitrack_d.pose.orientation.x = pose->pose.orientation.x;
  actual_optitrack_d.pose.orientation.y = pose->pose.orientation.y;
  actual_optitrack_d.pose.orientation.z = pose->pose.orientation.z;
  actual_optitrack_d.pose.orientation.w = pose->pose.orientation.w;
  
  optitrack_d = true;
}

void optitrackECallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_optitrack_e.time = sec + (nsec / 1000000000) - init_time;

  actual_optitrack_e.pose.position.x = pose->pose.position.x;
  actual_optitrack_e.pose.position.y = pose->pose.position.y;
  actual_optitrack_e.pose.position.z = pose->pose.position.z;

  actual_optitrack_e.pose.orientation.x = pose->pose.orientation.x;
  actual_optitrack_e.pose.orientation.y = pose->pose.orientation.y;
  actual_optitrack_e.pose.orientation.z = pose->pose.orientation.z;
  actual_optitrack_e.pose.orientation.w = pose->pose.orientation.w;
  
  optitrack_e = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_file_generator");

  ros::NodeHandle n;

  // PARAMS
  n.getParam("pose_file_generator/pose_information", pose_information);
  n.getParam("pose_file_generator/get_odom", get_odom);
  n.getParam("pose_file_generator/get_amcl", get_amcl);
  n.getParam("pose_file_generator/get_optitrack", get_optitrack);

  // SUBSCRIBERS
  ros::Subscriber pose_a_sub = n.subscribe("/kobuki_a/odom", 1000, poseACallback);
  ros::Subscriber pose_b_sub = n.subscribe("/kobuki_b/odom", 1000, poseBCallback);
  ros::Subscriber pose_c_sub = n.subscribe("/kobuki_c/odom", 1000, poseCCallback);
  ros::Subscriber pose_d_sub = n.subscribe("/kobuki_d/odom", 1000, poseDCallback);
  ros::Subscriber pose_e_sub = n.subscribe("/kobuki_e/odom", 1000, poseECallback);

  ros::Subscriber amcl_a_sub = n.subscribe("/kobuki_a/amcl_pose", 1000, amclACallback);
  ros::Subscriber amcl_b_sub = n.subscribe("/kobuki_b/amcl_pose", 1000, amclBCallback);
  ros::Subscriber amcl_c_sub = n.subscribe("/kobuki_c/amcl_pose", 1000, amclCCallback);
  ros::Subscriber amcl_d_sub = n.subscribe("/kobuki_d/amcl_pose", 1000, amclDCallback);
  ros::Subscriber amcl_e_sub = n.subscribe("/kobuki_e/amcl_pose", 1000, amclECallback);

  ros::Subscriber optitrack_a_sub = n.subscribe("/optitrack/kobuki_a/pose", 1000, optitrackACallback);
  ros::Subscriber optitrack_b_sub = n.subscribe("/optitrack/kobuki_b/pose", 1000, optitrackBCallback);
  ros::Subscriber optitrack_c_sub = n.subscribe("/optitrack/kobuki_c/pose", 1000, optitrackCCallback);
  ros::Subscriber optitrack_d_sub = n.subscribe("/optitrack/kobuki_d/pose", 1000, optitrackDCallback);
  ros::Subscriber optitrack_e_sub = n.subscribe("/optitrack/kobuki_e/pose", 1000, optitrackECallback);

  folder_name = current_date();
       

  std::string directory = folder_direction + "/" + folder_name;

  /*std::string odom_type = "odom_";
  std::string amcl_type = "amcl_";
  std::string optitrack_type = "optitrack_";*/

  ros::Rate loop_rate(10);

  std::ofstream outfile_a;
  std::ofstream outfile_b;
  std::ofstream outfile_c;
  std::ofstream outfile_d;
  std::ofstream outfile_e;

  std::ofstream outfile_a_amcl;
  std::ofstream outfile_b_amcl;
  std::ofstream outfile_c_amcl;
  std::ofstream outfile_d_amcl;
  std::ofstream outfile_e_amcl;

  std::ofstream outfile_a_optitrack;
  std::ofstream outfile_b_optitrack;
  std::ofstream outfile_c_optitrack;
  std::ofstream outfile_d_optitrack;
  std::ofstream outfile_e_optitrack;

  bool first_loop = true;

  bool open_a = true;
  bool open_b = true;
  bool open_c = true;
  bool open_d = true;
  bool open_e = true;

  bool open_a_amcl = true;
  bool open_b_amcl = true;
  bool open_c_amcl = true;
  bool open_d_amcl = true;
  bool open_e_amcl = true;

  bool open_a_optitrack = true;
  bool open_b_optitrack = true;
  bool open_c_optitrack = true;
  bool open_d_optitrack = true;
  bool open_e_optitrack = true;

  //tf2_ros::Buffer tf_buffer_b;
  //tf2::TransformListener listener;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while (ros::ok())
  {

    if(first_loop)
    {
      init_time = ros::Time::now().toSec();
      
      printf("Init_time: %f", init_time);

      std::string mkdir_command = "mkdir -p ";

      std::string create_directory = mkdir_command + directory;

      char_arr = &create_directory[0]; // str to char[]
  
      system(char_arr); // Creating a directory

      if(get_odom)
      {
        create_directory.clear();
        create_directory = mkdir_command + directory + "/" + odom_folder;

        char_odom = &create_directory[0]; // str to char[]
    
        system(char_odom); // Creating a directory for odom files
      }
      if(get_amcl)
      {
        create_directory.clear();
        create_directory = mkdir_command + directory + "/" + amcl_folder;

        char_amcl = &create_directory[0]; // str to char[]
    
        system(char_amcl); // Creating a directory for amcl files
      }
      if(get_optitrack)
      {
        create_directory.clear();
        create_directory = mkdir_command + directory + "/" + optitrack_folder;

        char_optitrack = &create_directory[0]; // str to char[]
    
        system(char_optitrack); // Creating a directory for optitrack files
      }
      /*if (status == -1)
      {
        printf("__________________________________Error");
      } else {
        printf("__________________________________Directory created");
      }*/


    } else {

      
      if(get_odom) // odometry
      {
        if(pose_a)
        {
          if(open_a)
          {
            std::string file_name_a = directory + "/" + odom_folder + "/" + "kobuki_a.txt";
            outfile_a.open(file_name_a, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_a << "  Time        X         Y         YAW" << std::endl;
            open_a = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "kobuki_a/odom",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_pose_a.pose, actual_pose_a.pose, transformStamped);
          outfile_a << std::to_string(actual_pose_a.time) << ", " << std::to_string(actual_pose_a.pose.position.x) << ", " << std::to_string(actual_pose_a.pose.position.y) << ", " << std::to_string(getYaw(actual_pose_a)) << std::endl;
          pose_a = false;
        }

        if(pose_b)
        {
          if(open_b)
          {
            std::string file_name_b = directory + "/" + odom_folder + "/"  + "kobuki_b.txt";
            outfile_b.open(file_name_b, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_b << "  Time        X         Y         YAW" << std::endl;
            open_b = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "kobuki_b/odom",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_pose_b.pose, actual_pose_b.pose, transformStamped);
          outfile_b << std::to_string(actual_pose_b.time) << ", " << std::to_string(actual_pose_b.pose.position.x) << ", " << std::to_string(actual_pose_b.pose.position.y) << ", " << std::to_string(getYaw(actual_pose_b)) << std::endl;
          pose_b = false;
        }

        if(pose_c)
        {
          if(open_c)
          {
            std::string file_name_c = directory + "/" + odom_folder + "/"  + "kobuki_c.txt";
            outfile_c.open(file_name_c, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_c << "  Time        X         Y         YAW" << std::endl;
            open_c = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "kobuki_c/odom",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_pose_c.pose, actual_pose_c.pose, transformStamped);
          outfile_c << std::to_string(actual_pose_c.time) << ", " << std::to_string(actual_pose_c.pose.position.x) << ", " << std::to_string(actual_pose_c.pose.position.y) << ", " << std::to_string(getYaw(actual_pose_c)) << std::endl;
          pose_c = false;
        }

        if(pose_d)
        {
          if(open_d)
          {
            std::string file_name_d = directory + "/" + odom_folder + "/"  + "kobuki_d.txt";
            outfile_d.open(file_name_d, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_d << "  Time        X         Y         YAW" << std::endl;
            open_d = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "kobuki_d/odom",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_pose_d.pose, actual_pose_d.pose, transformStamped);

          outfile_d << std::to_string(actual_pose_d.time) << ", " << std::to_string(actual_pose_d.pose.position.x) << ", " << std::to_string(actual_pose_d.pose.position.y) << ", " << std::to_string(getYaw(actual_pose_d)) << std::endl;
          pose_d = false;
        }

        if(pose_e)
        {
          if(open_e)
          {
            std::string file_name_e = directory + "/" + odom_folder + "/"  + "kobuki_e.txt";
            outfile_e.open(file_name_e, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_e << "  Time        X         Y         YAW" << std::endl;
            open_e = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "kobuki_e/odom",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_pose_e.pose, actual_pose_e.pose, transformStamped);

          outfile_e << std::to_string(actual_pose_e.time) << ", " << std::to_string(actual_pose_e.pose.position.x) << ", " << std::to_string(actual_pose_e.pose.position.y) << ", " << std::to_string(getYaw(actual_pose_e)) << std::endl;
          pose_e = false;
        }

      }
      if(get_amcl) // amcl
      {
        if(amcl_a)
        {
          if(open_a_amcl)
          {
            std::string file_name_a = directory + "/" + amcl_folder + "/"  + "kobuki_a.txt";
            outfile_a_amcl.open(file_name_a, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_a_amcl << "   Time        X         Y         YAW" << std::endl;
            open_a_amcl = false;
          }
          outfile_a_amcl << std::to_string(actual_amcl_a.time) << ", " << std::to_string(actual_amcl_a.pose.position.x) << ", " << std::to_string(actual_amcl_a.pose.position.y) << ", " << std::to_string(getYaw(actual_amcl_a)) << std::endl;
          amcl_a = false;
        }

        if(amcl_b)
        {
          if(open_b_amcl)
          {
            std::string file_name_b = directory + "/" + amcl_folder + "/"  + "kobuki_b.txt";
            outfile_b_amcl.open(file_name_b, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_b_amcl << "   Time        X         Y         YAW" << std::endl;
            open_b_amcl = false;
          }
          outfile_b_amcl << std::to_string(actual_amcl_b.time) << ", " << std::to_string(actual_amcl_b.pose.position.x) << ", " << std::to_string(actual_amcl_b.pose.position.y) << ", " << std::to_string(getYaw(actual_amcl_b)) << std::endl;
          amcl_b = false;
        }

        if(amcl_c)
        {
          if(open_c_amcl)
          {
            std::string file_name_c = directory + "/" + amcl_folder + "/"  + "kobuki_c.txt";
            outfile_c_amcl.open(file_name_c, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_c_amcl << "   Time        X         Y         YAW" << std::endl;
            open_c_amcl = false;
          }
          outfile_c_amcl << std::to_string(actual_amcl_c.time) << ", " << std::to_string(actual_amcl_c.pose.position.x) << ", " << std::to_string(actual_amcl_c.pose.position.y) << ", " << std::to_string(getYaw(actual_amcl_c)) << std::endl;
          amcl_c = false;
        }

        if(amcl_d)
        {
          if(open_d_amcl)
          {
            std::string file_name_d = directory + "/" + amcl_folder + "/"  + "kobuki_d.txt";
            outfile_d_amcl.open(file_name_d, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_d_amcl << "   Time        X         Y         YAW" << std::endl;
            open_d_amcl = false;
          }
          outfile_d_amcl << std::to_string(actual_amcl_d.time) << ", " << std::to_string(actual_amcl_d.pose.position.x) << ", " << std::to_string(actual_amcl_d.pose.position.y) << ", " << std::to_string(getYaw(actual_amcl_d)) << std::endl;
          amcl_d = false;
        }

        if(amcl_e)
        {
          if(open_e_amcl)
          {
            std::string file_name_e = directory + "/" + amcl_folder + "/"  + "kobuki_e.txt";
            outfile_e_amcl.open(file_name_e, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_e_amcl << "   Time        X         Y         YAW" << std::endl;
            open_e_amcl = false;
          }
          outfile_e_amcl << std::to_string(actual_amcl_e.time) << ", " << std::to_string(actual_amcl_e.pose.position.x) << ", " << std::to_string(actual_amcl_e.pose.position.y) << ", " << std::to_string(getYaw(actual_amcl_e)) << std::endl;
          amcl_e = false;
        }

      }
      if(get_optitrack) // optitrack
      {
        if(optitrack_a)
        {
          if(open_a_optitrack)
          {
            std::string file_name_a = directory + "/" + optitrack_folder + "/"  + "kobuki_a.txt";
            outfile_a_optitrack.open(file_name_a, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_a_optitrack << "  Time        X         Y         YAW" << std::endl;
            open_a_optitrack = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "optitrack",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_optitrack_a.pose, actual_optitrack_a.pose, transformStamped);

          outfile_a_optitrack << std::to_string(actual_optitrack_a.time) << ", " << std::to_string(actual_optitrack_a.pose.position.x) << ", " << std::to_string(actual_optitrack_a.pose.position.y) << ", " << std::to_string(getYaw(actual_optitrack_a)) << std::endl;
          optitrack_a = false;
        }

        if(optitrack_b)
        {
          if(open_b_optitrack)
          {
            std::string file_name_b = directory + "/" + optitrack_folder + "/"  + "kobuki_b.txt";
            outfile_b_optitrack.open(file_name_b, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_b_optitrack << "  Time        X         Y         YAW" << std::endl;
            open_b_optitrack = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "optitrack",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_optitrack_b.pose, actual_optitrack_b.pose, transformStamped);

          outfile_b_optitrack << std::to_string(actual_optitrack_b.time) << ", " << std::to_string(actual_optitrack_b.pose.position.x) << ", " << std::to_string(actual_optitrack_b.pose.position.y) << ", " << std::to_string(getYaw(actual_optitrack_b)) << std::endl;
          optitrack_b = false;
        }

        if(optitrack_c)
        {
          if(open_c_optitrack)
          {
            std::string file_name_c = directory + "/" + optitrack_folder + "/"  + "kobuki_c.txt";
            outfile_c_optitrack.open(file_name_c, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_c_optitrack << "  Time        X         Y         YAW" << std::endl;
            open_c_optitrack = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "optitrack",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_optitrack_c.pose, actual_optitrack_c.pose, transformStamped);

          outfile_c_optitrack << std::to_string(actual_optitrack_c.time) << ", " << std::to_string(actual_optitrack_c.pose.position.x) << ", " << std::to_string(actual_optitrack_c.pose.position.y) << ", " << std::to_string(getYaw(actual_optitrack_c)) << std::endl;
          optitrack_c = false;
        }

        if(optitrack_d)
        {
          if(open_d_optitrack)
          {
            std::string file_name_d = directory + "/" + optitrack_folder + "/"  + "kobuki_d.txt";
            outfile_d_optitrack.open(file_name_d, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_d_optitrack << "  Time        X         Y         YAW" << std::endl;
            open_d_optitrack = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "optitrack",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_optitrack_d.pose, actual_optitrack_d.pose, transformStamped);

          outfile_d_optitrack << std::to_string(actual_optitrack_d.time) << ", " << std::to_string(actual_optitrack_d.pose.position.x) << ", " << std::to_string(actual_optitrack_d.pose.position.y) << ", " << std::to_string(getYaw(actual_optitrack_d)) << std::endl;
          optitrack_d = false;
        }

        if(optitrack_e)
        {
          if(open_e_optitrack)
          {
            std::string file_name_e = directory + "/" + optitrack_folder + "/"  + "kobuki_e.txt";
            outfile_e_optitrack.open(file_name_e, std::fstream::in | std::fstream::out | std::fstream::app); // app -> open the file and set the cursor to the last position
            outfile_e_optitrack << "  Time        X         Y         YAW" << std::endl;
            open_e_optitrack = false;
          }

          geometry_msgs::TransformStamped transformStamped;
          try{
            transformStamped = tfBuffer.lookupTransform("map", "optitrack",ros::Time(0));
          } catch(...){
            printf("Can't transform\n");
          }
          /*printf("translation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

          printf("rotation:\n");
          printf("    x: %5.3f\n    y: %5.3f\n    z: %5.3f\n    w: %5.3f\n\n", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);*/
          tf2::doTransform(actual_optitrack_e.pose, actual_optitrack_e.pose, transformStamped);

          outfile_e_optitrack << std::to_string(actual_optitrack_e.time) << ", " << std::to_string(actual_optitrack_e.pose.position.x) << ", " << std::to_string(actual_optitrack_e.pose.position.y) << ", " << std::to_string(getYaw(actual_optitrack_e)) << std::endl;
          optitrack_e = false;
        }

      }    
    }

    ros::spinOnce();

    loop_rate.sleep();

    first_loop = false;
    
  }

  outfile_a.close();
  outfile_b.close();
  outfile_c.close();
  outfile_d.close();
  outfile_e.close();

  outfile_a_amcl.close();
  outfile_b_amcl.close();
  outfile_c_amcl.close();
  outfile_d_amcl.close();
  outfile_e_amcl.close();

  outfile_a_optitrack.close();
  outfile_b_optitrack.close();
  outfile_c_optitrack.close();
  outfile_d_optitrack.close();
  outfile_e_optitrack.close();

  return 0;
}


double getYaw(Pose pose)
{
  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

std::string current_date(){
    time_t now = time(0);

    tm *ltm = localtime(&now);

    std::string date;

    date = std::to_string(1900 + ltm->tm_year) + "-" + std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_hour) + ":" + std::to_string(ltm->tm_min);
    //ltm->tm_sec //if we want the seconds
    return date;
}