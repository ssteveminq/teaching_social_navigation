#include "ros/ros.h"
#include "manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
// #include "tf/StampedTransform.h"
#include "tf/LinearMath/Transform.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
//#include "tf/Vector3.h"
#include <Eigen/Dense>
#include <sstream>
#include "cba_msgs/CBA_NavInfo.h"
#include <boost/thread/thread.hpp>


using namespace Eigen;


bool boolReceive=true;
ELMClassifier     elmclassifier_;

ELMClassifier     wow2;

std_msgs::String GUI_msg;
MapParam        CurrentMap;
MapParam         MatlabMap;

int  CMD_GUI=0;
char sz[18];

// void CmdStrCallback(const std_msgs::String::ConstPtr& msg);
// void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg);
// void NavInfo_Callback(const cba_msgs::CBA_NavInfo::ConstPtr& msg);






void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
//  std::cout <<"Helloworld" << std::endl; 
}

void callback2(const ros::TimerEvent&)
{
  ROS_INFO("Callback 2 triggered");

}

int main(int argc, char **argv)
{

//////////////////
  ros::init(argc, argv, "talker");
  CBAManager      m_Manager;
  // ros::Rate r(5);
  m_Manager.pMapParam=&CurrentMap;
  m_Manager.pClassifier=&elmclassifier_;
  //This should be called after the 
  m_Manager.IntializeROS_publisher();
  //m_Manager.LoadDataFile();
 	// std::cout <<"Helloworld" << std::endl; 
  ros::Publisher  Matlab_Pub;
  ros::Publisher  Map_Pub;
  ros::Subscriber cmd_sub;
  ros::Subscriber cmd_sub2;
  ros::Subscriber grid_sub;   
  ros::Subscriber grid_sub2; 
  ros::Subscriber unitgoal_sub; 
  ros::Subscriber mdpsol_sub;
  ros::Subscriber global_pos_sub;
  ros::Subscriber Odometery_sub;
  ros::NodeHandle n;

  tf::TransformListener listener;

 //  ros::Timer timer1 = n.createTimer(ros::Duration(0.5), callback1);
 //  Publisher
  //Subscriber
  cmd_sub2 = n.subscribe<std_msgs::Int8>("/CBA_cmd_int", 50,&CBAManager::CmdIntCallback,&m_Manager);
  grid_sub = n.subscribe<cba_msgs::CBA_NavInfo>("/CBA_featureV", 20, &CBAManager::NavInfo_Callback,&m_Manager);  
  unitgoal_sub = n.subscribe<std_msgs::Float32MultiArray>("/CBA_unit_goal", 10, &CBAManager::Unitgoal_Callback,&m_Manager);
  mdpsol_sub = n.subscribe<std_msgs::Int32MultiArray>("/MDP/Solution", 10, &CBAManager::mdpsol_Callback,&m_Manager);
  // Odometery_sub=n.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, hsrbodom_Callback);
  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &CBAManager::global_pose_callback,&m_Manager);
  m_Manager.HSR_Pub =n.advertise<std_msgs::Int8>("/CBA_action_cmd", 30);


  ros::Rate loop_rate(20);

  while (ros::ok())
  {

  
  	 ros::spinOnce();
     loop_rate.sleep();   
      // ros::spin();
   }
   ros::spin();
  return 0;
}





