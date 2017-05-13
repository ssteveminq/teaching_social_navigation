#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PointStamped.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>



#include <iostream>
#include <sstream>
#include <cmath>

using namespace grid_map;

std::vector<int> cur_coord(2);
std::vector<int> GoalVector_Rviz(2);
std::vector<int> Goal_Coord(2);

std::string WORLD_FRAME = "base_range_sensor_link";
std::string MAP_FRAME = "map_local";
std::string MAP_FRAME2 = "map2";

int x_size=140;
int y_size=140;
double unit_step=0.05;
double center_offset=0.0;


int x_size2=28;
int y_size2=28;
double unit_step2=0.25;




int Robotaction=0;
int cur_stid=0;

sensor_msgs::JointState joint_state;
std::vector<float> costmap(x_size*y_size);
std::vector<int> obsmap;
std::vector<int> pathmap;
std::vector<int> localmap(x_size*y_size);
std::vector<double> hsrb_base_pose(3,0.0);


//function that convert from state id to 2d coordinates
std::vector<int> ConvertId2coord(int _stid)
{
  std::vector<int> tempcoord(2);
  
  tempcoord[0]= (int)_stid%x_size;
  tempcoord[1]= (int)_stid/x_size;

  std::cout<<"x:"<<tempcoord[0]<<" , "<<"y:"<<tempcoord[1]<<std::endl;

  return tempcoord;
}

//Update joint configuration
void UpdateJointState(int action)
{
   std::cout<<"Update Joint State"<<std::endl;

   //when firstly used
   if(joint_state.name.size()<2){

      joint_state.name.push_back("base_to_wheel_j0");
      joint_state.name.push_back("base_to_wheel_j1");
      joint_state.name.push_back("base_to_wheel_j2");
      joint_state.name.push_back("virtual_x");
      joint_state.name.push_back("virtual_y");
      joint_state.name.push_back("virtual_yaw");
      
      joint_state.position.push_back(0.0);
      joint_state.position.push_back(0.0);
      joint_state.position.push_back(0.0);
      joint_state.position.push_back(-0.3);
      joint_state.position.push_back(-0.3);
      joint_state.position.push_back(0.0);
    }
    //for other times - just update joint state position

      // std::cout<<"else"<<std::endl;

      if((abs(joint_state.position[3])<3.6) && (abs(joint_state.position[4])<5.4))
      {

        double unit_step=0.58;
        switch(action)
        {
          case 0: ROS_INFO("South");
            joint_state.position[4] -= unit_step;
            //joint_state.position[4] += 0.3;
          break;
          case 1:  ROS_INFO("East");
            joint_state.position[3] -= unit_step;
          break;
         case 2: ROS_INFO("North");
            joint_state.position[4] += unit_step;
          break;

          case 3: ROS_INFO("West");
            joint_state.position[3] += unit_step;
          break;
          default: ROS_INFO("Wrong action");

        }
       }
}

void UpdateJointStatebyID(int cur_stateid)
{
   std::cout<<"Update Joint State by Id"<<std::endl;
   //when firstly used
   if(joint_state.name.size()==0){

      joint_state.name.push_back("base_to_wheel_j0");
      joint_state.name.push_back("base_to_wheel_j1");
      joint_state.name.push_back("base_to_wheel_j2");
      joint_state.name.push_back("virtual_x");
      joint_state.name.push_back("virtual_y");
      joint_state.name.push_back("virtual_yaw");
      
      joint_state.position.push_back(0.0);
      joint_state.position.push_back(0.0);
      joint_state.position.push_back(0.0);
      joint_state.position.push_back(-0.3);
      joint_state.position.push_back(-0.3);
      joint_state.position.push_back(0.0);
    }
    
    // std::cout<<"else"<<std::endl;
    std::vector<int> cur_coord=ConvertId2coord(cur_stateid);

    if((abs(cur_coord[0])<x_size+1) && (abs(cur_coord[1])<y_size+1))
      {
          joint_state.position[3]=-unit_step*cur_coord[0]-center_offset;
          joint_state.position[4]=-unit_step*cur_coord[1]-center_offset;
      }
}

//Callback function for ros subscriber
void stateidCallback(const std_msgs::Int32::ConstPtr& msg)
{
    cur_stid=(int)(msg->data);
    std::cout<<"state id"<<cur_stid<<std::endl;
    UpdateJointStatebyID(cur_stid);
    return;
}
//Callback function for action
void actionCallback(const std_msgs::Int32::ConstPtr& msg)
{
        return;
}

//Callback function for map
void costmapCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //ROS_INFO("int msg");
   // ROS_INFO("Cur costmap cmd :%d ",msg->data);
    std::cout<<"Receive costmap"<<std::endl;
    costmap.resize(msg->data.size());

      for(int i(0);i<msg->data.size();i++){
         costmap[i]=(msg->data)[i];
      }

     return;
}

//Callback function for map
void localmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
   //  ROS_INFO("int msg");
   // // ROS_INFO("Cur costmap cmd :%d ",msg->data);
   //  std::cout<<"Receive localmap"<<std::endl;
   
     localmap.resize(msg->data.size());

      for(int i(0);i<msg->data.size();i++){
         localmap[i]=(msg->data)[i];
      }

     return;
}


void obsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
      std::cout<<"Receive obsmap"<<std::endl;
      obsmap.resize(msg->data.size());

      for(int i(0);i<msg->data.size();i++){
         obsmap[i]=(msg->data)[i];
      }

       return;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    hsrb_base_pose[0]=msg->pose.pose.position.x;
    hsrb_base_pose[1]=msg->pose.pose.position.y;
    hsrb_base_pose[2]=msg->pose.pose.orientation.z;
    // hsrb_base_pose[0]=odom_msg->pose.pose.position.x;
    // hsrb_base_pose[1]=odom_msg->pose.pose.position.y;
 
    return;
}

  

  void pathCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
      //std::cout<<"Receive pathmap"<<std::endl;
      //ROS_INFO("Receive pathmap");
      pathmap.resize(msg->data.size());

      for(int i(0);i<msg->data.size();i++){
         pathmap[i]=(msg->data)[i];
      
         //std::cout<<pathmap[i]<<std::endl;
      }

        return;
}

 void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
      printf("Receive point\n");
      printf("x is %.3f, y is %.3f \n",msg->point.x,msg->point.y);

      GoalVector_Rviz[0]=msg->point.x;
      GoalVector_Rviz[0]=msg->point.y;
        return;
}
 

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_navigation_map");
  ros::NodeHandle nh("~");
 
  for(int i(0);i<x_size*y_size;i++)
    localmap[i]=0.000*i;

  //set Goal vector
  GoalVector_Rviz[0]=0.0;
  GoalVector_Rviz[1]=0.0;

  //Publisher
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 50, true);
  ros::Publisher publisher2 = nh.advertise<grid_map_msgs::GridMap>("grid_map2", 50, true);
    
  //Subscriber from despot
  ros::Subscriber Costmap_sub;
  ros::Subscriber Action_sub;
  ros::Subscriber StateId_sub;
  ros::Subscriber ObsId_sub;
  ros::Subscriber PathId_sub;
  ros::Subscriber Point_sub;
  ros::Subscriber Localmap_sub;
  ros::Subscriber Odom_sub;

  //Initialize ros subsriber
  //Costmap_sub   = nh.subscribe<std_msgs::Float32MultiArray>("/POMDP/costmap", 10,costmapCallback); 
  
  // Localmap_sub   = nh.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 10, localmapCallback); 
  Odom_sub = nh.subscribe<nav_msgs::Odometry>("/hsrb/odom", 10, odomCallback);
  Localmap_sub   = nh.subscribe<nav_msgs::OccupancyGrid>("/local_map_navigation_map/local_map", 10, localmapCallback); 
  // Costmap_sub    = nh.subscribe<std_msgs::Float32MultiArray>("/POMDP/costmap", 10,costmapCallback); 
  // Action_sub     = nh.subscribe<std_msgs::Int32>("/POMDP/action", 10,actionCallback);
  // StateId_sub    = nh.subscribe<std_msgs::Int32>("/POMDP/stateid", 10, stateidCallback);
  ObsId_sub      = nh.subscribe<std_msgs::Int32MultiArray>("/MDP/costmap", 10,obsCallback);
  PathId_sub     = nh.subscribe<std_msgs::Int32MultiArray>("/MDP/path", 10, pathCallback);
  Point_sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, pointCallback);


  //Tf function for connecting robot frame to world
  tf::TransformBroadcaster    world_to_map_br;
  tf::TransformBroadcaster    world_to_map_br2;
  tf::Transform               world_to_map_transform; 
  tf::Transform               world_to_map_transform2; 

  // Create grid map.
  GridMap map({"type"});
  map.setFrameId("map_local");
  //map.setGeometry(Length(x_size*unit_step, y_size*unit_step), unit_step ,Position(-0.5*x_size*unit_step,-0.5*y_size*unit_step));
  map.setGeometry(Length(x_size*unit_step, y_size*unit_step), unit_step ,Position(0,0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  map.getLength().x(), map.getLength().y(),
  map.getSize()(0), map.getSize()(1));

  GridMap map2({"elevation"});
  map2.setFrameId("map_local");
  //map2.setGeometry(Length(x_size*unit_step, y_size*unit_step), unit_step ,Position(-0.5*x_size*unit_step,-0.5*y_size*unit_step));
  map2.setGeometry(Length(x_size2*unit_step2, y_size2*unit_step2), unit_step2 ,Position(-0,-0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
  map2.getLength().x(), map2.getLength().y(),
  map2.getSize()(0), map2.getSize()(1));
  
 ros::Rate rate(30);
  
 while (nh.ok()) {
  map.clearAll();
  map.setTimestamp(ros::Time::now().toNSec());
  world_to_map_transform.setOrigin(tf::Vector3(0.0, 0.0, -0.0) );
  tf::Quaternion q;
  //float yaw =hsrb_base_pose[2];
   float yaw =hsrb_base_pose[2]+3.141592;

  q.setRPY(0, 0, yaw);
  world_to_map_transform.setRotation(q);
  world_to_map_br.sendTransform(tf::StampedTransform(world_to_map_transform, ros::Time::now(), WORLD_FRAME, MAP_FRAME));

  // world_to_map_transform2.setOrigin( tf::Vector3(0, 0, 0.0) );
  // tf::Quaternion q2;
  // float yaw2 = 0.0;
  // q2.setRPY(0, 0.0, 0);
  // world_to_map_transform2.setRotation(q2);
  // world_to_map_br2.sendTransform(tf::StampedTransform(world_to_map_transform2, ros::Time::now(), WORLD_FRAME, MAP_FRAME2));

  // Add data to grid map.
  //ros::Time time = ros::Time::now();
  grid_map::Matrix& data = map["type"];
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const int i = iterator.getLinearIndex();

   data(i)=50*localmap[i];
      
    for(int k=0;k<pathmap.size();k++){
      if(i==pathmap[k])
        data(i)=30.0;
       }
   }

   grid_map::Matrix& data2 = map2["elevation"];
   for (grid_map::GridMapIterator iterator2(map2); !iterator2.isPastEnd(); ++iterator2) {
    const int j = iterator2.getLinearIndex();

    data2(j)=localmap[j];
      
   for(int k=0;k<pathmap.size();k++){
        
        if(j==pathmap[k])
          data2(j)=80;
       }
   }
  

    //Publish map data
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);

    map2.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message2;
    grid_map::GridMapRosConverter::toMessage(map2, message2);
    publisher.publish(message2);

    // Wait for next cycle.
    rate.sleep();
    ros::spinOnce(); // this is where the magic happens!!

  }

  return 0;
}

