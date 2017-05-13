#include "ros/ros.h"
#include "MDPmanager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "srBSpline.h"

using namespace Eigen;

bool boolSolve=false;


//MDPManager problemmanager;
MapParam   mapParam;


static int  Receive_count=0;
int x_size=Grid_Num_X;
int y_size=Grid_Num_Y;
double unit_step=0.25;
double center_offset=0.3;
vector<int>    localoccupancy(Grid_Num_X*Grid_Num_Y,0);

std::vector<int> ConvertId2coord(int _stid)
{
  // std::vector<int> tempcoord(2);
  
  // tempcoord[0]= (int)_stid%x_size;
  // tempcoord[1]= (int)_stid/x_size;

  // std::cout<<"x:"<<tempcoord[0]<<" , "<<"y:"<<tempcoord[1]<<std::endl;

  // return tempcoord;
}

void local_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
   //  //Receive map parameters
   //  if(Receive_count==0)
   //  {
   //    ROS_INFO("width : %d" ,msg->info.width);
   //    ROS_INFO("height : %d" ,msg->info.height);
   //    ROS_INFO("resolution : %lf" ,msg->info.resolution);
   //    Receive_count++;

   //    x_size=(int) msg->info.width;
   //    y_size=(int) msg->info.height;  

   //  }

   // double base_origin_x =msg->info.origin.position.x;
   // double base_origin_y =msg->info.origin.position.y;

   // //ROS_INFO("origin x: %lf, y : %lf",base_origin_x,base_origin_y);
   // //ROS_INFO("int msg");
   //  localoccupancy.resize(msg->data.size());

   // for(int i(0);i<msg->data.size();i++){
   //      localoccupancy[i]=(msg->data)[i];
   // }
   
   // ReceiveData++;


   // problemmanager.updateMap(localoccupancy);



  // ROS_INFO("ReeiveData : %d", ReceiveData);

}

//Update goal position of hsrb (clicked point)
void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
   //    printf("Receive point\n");
      
   //    GoalVector[0]=msg->point.x-Map_orig_Vector[0];
   //    GoalVector[1]=msg->point.y-Map_orig_Vector[1];
            
   //    // CurVector[0]=9;
   //    // CurVector[1]=9;

   //    Global2MapCoord(CurVector,cur_coord);
   //    Global2MapCoord(GoalVector,Goal_Coord);
 
   // //   problemmanager.updateMap(localoccupancy,cur_coord,Goal_Coord);

   //    // printf("x index is %.3f, y index is %.3f \n",Goal_Coord[0],Goal_Coord[1]);  
   //    boolSolve=true;

   //    return;
}

//Update base position of hsrb
void basepositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  //ROS_INFO("base position msg");
   // Map_orig_Vector[0]= msg->point.x-3.5;
   // Map_orig_Vector[1]= msg->point.y-3.5;

   // CurVector[0]= msg->point.x;
   // CurVector[1]= msg->point.y;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MDP_planner");
  
  MDPManager problemmanager; 
  
  //problemmanager.setRosObj(&nodeObj);
   problemmanager.setPMapParam(&mapParam);

  // ros::Rate r(5);
  ros::Subscriber Point_sub;  
  ros::Subscriber CurPos_sub;
  ros::Subscriber Localmap_sub;

  ros::NodeHandle n;
  
  Point_sub    = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &MDPManager::ClikedpointCallback,&problemmanager);
  Localmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/local_map_navigation_map/local_map", 30, &MDPManager::Local_mapCallback,&problemmanager); 
  
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
	  
    
     if(problemmanager.m_boolSolve)
     {
        printf("Begin to solve\n");
        problemmanager.pathPublish();
        problemmanager.MDPsolve(); 
        problemmanager.printPath(); 
        problemmanager.generatePath();
        
        printf("End solve\n");
        problemmanager.m_boolSolve=false;
     }

  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




