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

#include <fuzzymar_multi_robot/time_task.h>
#include <fuzzymar_multi_robot/robotParameters.h>

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

struct TaskEnd {
  int id_task;
  double time_completion;
  float deadline;
  float utility_max;
  uint8_t total_ports;
};

std::vector<TaskEnd> task_end_vector;

// ODOM
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

bool get_odom = true;
bool get_amcl = false;
bool get_optitrack = false;

char* char_arr;
char* char_odom;
char* char_amcl;
char* char_optitrack;
char* char_mission;

double init_time;
bool start_mission = false;
bool task_end = false;

std::vector<fuzzymar_multi_robot::robotParameters> rob_param_vec;
bool robot_param_received = false;

std::string folder_direction = "/home/tonitauler/ROS/kobuki_ws/src/file_generator/data";
std::string folder_name = "pere";
std::string odom_folder = "odom";
std::string amcl_folder = "amcl";
std::string mission_folder = "mission";
std::string optitrack_folder = "optitrack";
std::string file_format = ".txt"; // default .txt

// FUNCTIONS
double getYaw(Pose pose);
std::string current_date();
void writeFile(bool need_transform, Pose actual_pose, bool* open_file, std::ofstream* outfile, tf2_ros::Buffer* tfBuffer, tf2_ros::TransformListener* tfListener/*(tf2_ros::Buffer* tfBuffer)*/, std::string file_name, std::string map_frame, std::string loc_frame);
Pose updatePosition(const nav_msgs::Odometry::ConstPtr& pose);
Pose updatePositionAMCL(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
Pose updatePositionOptitrack(const geometry_msgs::PoseStamped::ConstPtr& pose);

void timeInfoCallback(const fuzzymar_multi_robot::time_task::ConstPtr& msg)
{
  
  TaskEnd aux_task_end;

  if(msg->id_task == 0)
  {
    init_time = msg->sec + (msg->nsec/1000000000.0);
    //printf("Mission starts at: %f\n", init_time);
    start_mission = true;
  } else {
    //printf("Task_%i finish at sec: %f nsec: %f\n", msg->id_task, double(msg->sec), double(msg->nsec));
    aux_task_end.id_task = msg->id_task;
    aux_task_end.time_completion = (msg->sec + (msg->nsec/1000000000.0)) - init_time;
    aux_task_end.deadline = msg->deadline;
    aux_task_end.utility_max = msg->utility_max;
    aux_task_end.total_ports = msg->total_ports;

    task_end_vector.push_back(aux_task_end);

    task_end = true;
  }

}

void poseACallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  actual_pose_a = updatePosition(pose);

  pose_a = true;
}

void poseBCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  actual_pose_b = updatePosition(pose);

  pose_b = true;
}

void poseCCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  actual_pose_c = updatePosition(pose);
  
  pose_c = true;
}

void poseDCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  actual_pose_d = updatePosition(pose);

  pose_d = true;
}

void poseECallback(const nav_msgs::Odometry::ConstPtr& pose)
{
  
  actual_pose_e = updatePosition(pose);
  
  pose_e = true;
}

void amclACallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  actual_amcl_a = updatePositionAMCL(pose);
  
  amcl_a = true;
}

void amclBCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  actual_amcl_b = updatePositionAMCL(pose);
  
  amcl_b = true;
}

void amclCCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  actual_amcl_c = updatePositionAMCL(pose);
  
  amcl_c = true;
}

void amclDCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  actual_amcl_d = updatePositionAMCL(pose);
  
  amcl_d = true;
}

void amclECallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  actual_amcl_e = updatePositionAMCL(pose);
  
  amcl_e = true;
}

void optitrackACallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  actual_optitrack_a = updatePositionOptitrack(pose);
  
  optitrack_a = true;
}

void optitrackBCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  actual_optitrack_b = updatePositionOptitrack(pose);
  
  optitrack_b = true;
}

void optitrackCCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  actual_optitrack_c = updatePositionOptitrack(pose);
  
  optitrack_c = true;
}

void optitrackDCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  actual_optitrack_d = updatePositionOptitrack(pose);
  
  optitrack_d = true;
}

void optitrackECallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  
  actual_optitrack_e = updatePositionOptitrack(pose);
  
  optitrack_e = true;
}

void robParamCallback(const fuzzymar_multi_robot::robotParameters::ConstPtr& msg)
{
  fuzzymar_multi_robot::robotParameters rob_param_aux;

  rob_param_aux.id_robot = msg->id_robot;
  rob_param_aux.max_vel = msg->max_vel;
  rob_param_aux.UDD_factor = msg->UDD_factor;
  rob_param_aux.alpha_utility = msg->alpha_utility;
  rob_param_aux.beta_distance = msg->beta_distance;
  rob_param_aux.gamma_ports = msg->gamma_ports;
  rob_param_aux.selection_task = msg->selection_task;
  rob_param_aux.calc_stim_time = msg->calc_stim_time;
  rob_param_aux.calc_stim_way = msg->calc_stim_way;
  rob_param_aux.sdl_method = msg->sdl_method;
  rob_param_aux.agregation_type = msg->agregation_type;
  rob_param_aux.w1 = msg->w1;
  rob_param_aux.w2 = msg->w2;
  rob_param_aux.w3 = msg->w3;


  rob_param_vec.push_back(rob_param_aux);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_file_generator");

  ros::NodeHandle n;

  // PARAMS
  n.getParam("pose_file_generator/get_odom", get_odom);
  n.getParam("pose_file_generator/get_amcl", get_amcl);
  n.getParam("pose_file_generator/get_optitrack", get_optitrack);
  n.getParam("pose_file_generator/file_format", file_format);

  // SUBSCRIBERS
  ros::Subscriber mission_sub = n.subscribe("/time_information", 10, timeInfoCallback);

  ros::Subscriber pose_a_sub = n.subscribe("/kobuki_a/odom", 10, poseACallback);
  ros::Subscriber pose_b_sub = n.subscribe("/kobuki_b/odom", 10, poseBCallback);
  ros::Subscriber pose_c_sub = n.subscribe("/kobuki_c/odom", 10, poseCCallback);
  ros::Subscriber pose_d_sub = n.subscribe("/kobuki_d/odom", 10, poseDCallback);
  ros::Subscriber pose_e_sub = n.subscribe("/kobuki_e/odom", 10, poseECallback);

  ros::Subscriber amcl_a_sub = n.subscribe("/kobuki_a/amcl_pose", 1, amclACallback);
  ros::Subscriber amcl_b_sub = n.subscribe("/kobuki_b/amcl_pose", 1, amclBCallback);
  ros::Subscriber amcl_c_sub = n.subscribe("/kobuki_c/amcl_pose", 1, amclCCallback);
  ros::Subscriber amcl_d_sub = n.subscribe("/kobuki_d/amcl_pose", 1, amclDCallback);
  ros::Subscriber amcl_e_sub = n.subscribe("/kobuki_e/amcl_pose", 1, amclECallback);

  ros::Subscriber optitrack_a_sub = n.subscribe("/optitrack/kobuki_a/pose", 100, optitrackACallback);
  ros::Subscriber optitrack_b_sub = n.subscribe("/optitrack/kobuki_b/pose", 100, optitrackBCallback);
  ros::Subscriber optitrack_c_sub = n.subscribe("/optitrack/kobuki_c/pose", 100, optitrackCCallback);
  ros::Subscriber optitrack_d_sub = n.subscribe("/optitrack/kobuki_d/pose", 100, optitrackDCallback);
  ros::Subscriber optitrack_e_sub = n.subscribe("/optitrack/kobuki_e/pose", 100, optitrackECallback);

  ros::Subscriber rob_param_sub = n.subscribe("/rob_parameters_info", 10, robParamCallback);

  folder_name = current_date();
       
  std::string directory = folder_direction + "/" + folder_name;

  /*std::string odom_type = "odom_";
  std::string amcl_type = "amcl_";
  std::string optitrack_type = "optitrack_";*/

  ros::Rate loop_rate(10);

  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string optitrack_frame = "optitrack";

  std::string pikachu = "kobuki_a";
  std::string bulbasaur = "kobuki_b";
  std::string charmander = "kobuki_c";
  std::string ditto = "kobuki_d";
  std::string eevee = "kobuki_e";

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

  std::ofstream outfile_mission;
  std::ofstream outfile_rob_param;

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

  int count_end_task = 0;

  while (ros::ok())
  {

    if(first_loop)
    {

      std::string mkdir_command = "mkdir -p ";

      std::string create_directory = mkdir_command + directory;

      char_arr = &create_directory[0]; // str to char[]
  
      system(char_arr); // Creating a directory

      create_directory.clear();
      create_directory = mkdir_command + directory + "/" + mission_folder;

      char_mission = &create_directory[0];

      system(char_mission);

      outfile_mission.open(directory + "/" + mission_folder + "/mission_log" + file_format, std::fstream::in | std::fstream::out | std::fstream::app);
      outfile_mission << "Task_ID,completion time,deadline,U_max,total_ports" << std::endl;

      outfile_rob_param.open(directory + "/" + mission_folder + "/robots_log" + file_format, std::fstream::in | std::fstream::out | std::fstream::app);
      outfile_rob_param << "Robot_ID,max_vel,UDD_factor,alpha_utility,beta_distance,gamma_ports,selection_task,calc_stim_time,calc_stim_way,sdl_method,agregation_type,w1,w2,w3" << std::endl;

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


    } else if(start_mission){

      
      if(task_end)
      {
        /*
        outfile_mission << std::to_string(aux_task_end.id_task) << "," << std::to_string(aux_task_end.time_completion) << "," << std::to_string(aux_task_end.deadline) << "," << std::to_string(aux_task_end.utility_max) << "," << std::to_string(aux_task_end.total_ports) << std::endl;
        task_end = false;*/
        for(count_end_task ; count_end_task < task_end_vector.size() ; count_end_task++)
        {
          outfile_mission << std::to_string(task_end_vector[count_end_task].id_task) << "," << std::to_string(task_end_vector[count_end_task].time_completion) << "," << std::to_string(task_end_vector[count_end_task].deadline) << "," << std::to_string(task_end_vector[count_end_task].utility_max) << "," << std::to_string(task_end_vector[count_end_task].total_ports) << std::endl;
        }
        task_end = false;

      }

      if(get_odom) // odometry
      {
        if(pose_a)
        {
          
          writeFile(true, actual_pose_a, &open_a, &outfile_a, &tfBuffer, &tfListener, directory + "/" + odom_folder + "/" + pikachu + file_format, map_frame, pikachu + "/" + odom_frame);
          
          pose_a = false;
        }

        if(pose_b)
        {
          
          writeFile(true, actual_pose_b, &open_b, &outfile_b, &tfBuffer, &tfListener, directory + "/" + odom_folder + "/" + bulbasaur + file_format, map_frame, bulbasaur + "/" + odom_frame);
          
          pose_b = false;
        }

        if(pose_c)
        {
          
          writeFile(true, actual_pose_c, &open_c, &outfile_c, &tfBuffer, &tfListener, directory + "/" + odom_folder + "/" + charmander + file_format, map_frame, charmander + "/" + odom_frame);
          
          pose_c = false;
        }

        if(pose_d)
        {
          
          writeFile(true, actual_pose_d, &open_d, &outfile_d, &tfBuffer, &tfListener, directory + "/" + odom_folder + "/" + ditto + file_format, map_frame, ditto + "/" + odom_frame);
          
          pose_d = false;
        }

        if(pose_e)
        {
          
          writeFile(true, actual_pose_e, &open_e, &outfile_e, &tfBuffer, &tfListener, directory + "/" + odom_folder + "/" + eevee + file_format, map_frame, eevee + "/" + odom_frame);
          
          pose_e = false;
        }

      }
      if(get_amcl) // amcl
      {
        if(amcl_a)
        {
          
          writeFile(false, actual_amcl_a, &open_a_amcl, &outfile_a_amcl, &tfBuffer, &tfListener, directory + "/" + amcl_folder + "/" + pikachu + file_format, "not_needed","not_needed");
          
          amcl_a = false;
        }

        if(amcl_b)
        {
          
          writeFile(false, actual_amcl_b, &open_b_amcl, &outfile_b_amcl, &tfBuffer, &tfListener, directory + "/" + amcl_folder + "/" + bulbasaur + file_format, "not_needed","not_needed");
          
          amcl_b = false;
        }

        if(amcl_c)
        {
          
          writeFile(false, actual_amcl_c, &open_c_amcl, &outfile_c_amcl, &tfBuffer, &tfListener, directory + "/" + amcl_folder + "/" + charmander + file_format, "not_needed","not_needed");
          
          amcl_c = false;
        }

        if(amcl_d)
        {
          
          writeFile(false, actual_amcl_d, &open_d_amcl, &outfile_d_amcl, &tfBuffer, &tfListener, directory + "/" + amcl_folder + "/" + ditto + file_format, "not_needed","not_needed");
          
          amcl_d = false;
        }

        if(amcl_e)
        {
          
          writeFile(false, actual_amcl_e, &open_e_amcl, &outfile_e_amcl, &tfBuffer, &tfListener, directory + "/" + amcl_folder + "/" + eevee + file_format, "not_needed","not_needed");
          
          amcl_e = false;
        }

      }
      if(get_optitrack) // optitrack
      {
        if(optitrack_a)
        {
          
          writeFile(true, actual_optitrack_a, &open_a_optitrack, &outfile_a_optitrack, &tfBuffer, &tfListener, directory + "/" + optitrack_folder + "/" + pikachu + file_format, map_frame, optitrack_frame);
          
          optitrack_a = false;
        }

        if(optitrack_b)
        {
          
          writeFile(true, actual_optitrack_b, &open_b_optitrack, &outfile_b_optitrack, &tfBuffer, &tfListener, directory + "/" + optitrack_folder + "/" + bulbasaur + file_format, map_frame, optitrack_frame);
          
          optitrack_b = false;
        }

        if(optitrack_c)
        {
          
          writeFile(true, actual_optitrack_c, &open_c_optitrack, &outfile_c_optitrack, &tfBuffer, &tfListener, directory + "/" + optitrack_folder + "/" + charmander + file_format, map_frame, optitrack_frame);
          
          optitrack_c = false;
        }

        if(optitrack_d)
        {
          
          writeFile(true, actual_optitrack_d, &open_d_optitrack, &outfile_d_optitrack, &tfBuffer, &tfListener, directory + "/" + optitrack_folder + "/" + ditto + file_format, map_frame, optitrack_frame);
          
          optitrack_d = false;
        }

        if(optitrack_e)
        {
          
          writeFile(true, actual_optitrack_e, &open_e_optitrack, &outfile_e_optitrack, &tfBuffer, &tfListener, directory + "/" + optitrack_folder + "/" + eevee + file_format, map_frame, optitrack_frame);
          
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

  for(int i = 0 ; i < rob_param_vec.size() ; i++)
  {

    outfile_rob_param << std::to_string(rob_param_vec[i].id_robot) << "," << std::to_string(rob_param_vec[i].max_vel) << "," << std::to_string(rob_param_vec[i].UDD_factor) << "," << std::to_string(rob_param_vec[i].alpha_utility) << "," << std::to_string(rob_param_vec[i].beta_distance) << "," << std::to_string(rob_param_vec[i].gamma_ports) << "," << rob_param_vec[i].selection_task << "," << std::to_string(rob_param_vec[i].calc_stim_time) << "," << std::to_string(rob_param_vec[i].calc_stim_way) << "," << std::to_string(rob_param_vec[i].sdl_method) << "," << std::to_string(rob_param_vec[i].agregation_type) << "," << std::to_string(rob_param_vec[i].w1) << "," << std::to_string(rob_param_vec[i].w2) << "," << std::to_string(rob_param_vec[i].w3)  << std::endl; 

  }

  outfile_mission.close();
  outfile_rob_param.close();
       

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

void writeFile(bool need_transform, Pose actual_pose, bool* open_file, std::ofstream* outfile, tf2_ros::Buffer* tfBuffer, tf2_ros::TransformListener* tfListener/*(tf2_ros::Buffer* tfBuffer)*/, std::string file_name, std::string map_frame, std::string loc_frame)
{
  double time_aux = 0.0;
  if(*open_file)
  {
    //printf("Capçalera, init_time: %f\n", init_time);
    outfile->open(file_name, std::fstream::in | std::fstream::out | std::fstream::app);
    *outfile << "Time,X,Y,YAW" << std::endl;
    *open_file = false;
  }

  if(need_transform)
  {
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer->lookupTransform(map_frame, loc_frame, ros::Time(0));
    } catch(...){
      printf("Can't transform\n");
    }
    //printf("Escric, init_time: %f\n", init_time);
    tf2::doTransform(actual_pose.pose, actual_pose.pose, transformStamped);
  }


  if(actual_pose.time - init_time > 0.0)
  {
    time_aux = actual_pose.time - init_time;
  }
  
  *outfile << std::to_string(time_aux) << "," << std::to_string(actual_pose.pose.position.x) << "," << std::to_string(actual_pose.pose.position.y) << "," << std::to_string(getYaw(actual_pose)) << std::endl;
  
}

Pose updatePosition(const nav_msgs::Odometry::ConstPtr& pose)
{

  Pose actual_pose_aux;

  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  //printf("                odom time -> sec: %f nsec: %f, init_time: %f\n", sec, nsec, init_time);
  actual_pose_aux.time = sec + (nsec / 1000000000.0);

  actual_pose_aux.pose.position.x = pose->pose.pose.position.x;
  actual_pose_aux.pose.position.y = pose->pose.pose.position.y;
  actual_pose_a.pose.position.z = pose->pose.pose.position.z;

  actual_pose_aux.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_pose_aux.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_pose_aux.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_pose_aux.pose.orientation.w = pose->pose.pose.orientation.w;

  return actual_pose_aux;

}

Pose updatePositionAMCL(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{

  Pose actual_amcl_aux;

  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  //printf("                amcl time -> sec: %f nsec: %f, init_time: %f\n", sec, nsec, init_time);
  actual_amcl_aux.time = sec + (nsec / 1000000000.0);

  actual_amcl_aux.pose.position.x = pose->pose.pose.position.x;
  actual_amcl_aux.pose.position.y = pose->pose.pose.position.y;
  actual_amcl_aux.pose.position.z = pose->pose.pose.position.z;

  actual_amcl_aux.pose.orientation.x = pose->pose.pose.orientation.x;
  actual_amcl_aux.pose.orientation.y = pose->pose.pose.orientation.y;
  actual_amcl_aux.pose.orientation.z = pose->pose.pose.orientation.z;
  actual_amcl_aux.pose.orientation.w = pose->pose.pose.orientation.w;

  return actual_amcl_aux;

}

Pose updatePositionOptitrack(const geometry_msgs::PoseStamped::ConstPtr& pose)
{

  Pose actual_optitrack_aux;

  double sec = pose->header.stamp.sec;
  double nsec = pose->header.stamp.nsec;
  actual_optitrack_aux.time = sec + (nsec / 1000000000.0);

  actual_optitrack_aux.pose.position.x = pose->pose.position.x;
  actual_optitrack_aux.pose.position.y = pose->pose.position.y;
  actual_optitrack_aux.pose.position.z = pose->pose.position.z;

  actual_optitrack_aux.pose.orientation.x = pose->pose.orientation.x;
  actual_optitrack_aux.pose.orientation.y = pose->pose.orientation.y;
  actual_optitrack_aux.pose.orientation.z = pose->pose.orientation.z;
  actual_optitrack_aux.pose.orientation.w = pose->pose.orientation.w;

  return actual_optitrack_aux;

}