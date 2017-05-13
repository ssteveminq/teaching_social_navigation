#include "ros/ros.h"
#include "manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include <Eigen/Dense>
#include <sstream>
#include "classifier_hsr/CBA_NavInfo.h"
#include <boost/thread/thread.hpp>


using namespace Eigen;


bool boolReceive=true;
ELMClassifier     elmclassifier_;

ELMClassifier     wow2;
CBAManager      m_Manager;
std_msgs::String GUI_msg;
MapParam        CurrentMap;
MapParam         MatlabMap;
int  CMD_GUI=0;

std::string host = "192.168.1.107";
std::string port = "1235";
std::string answer;
std::string sender;
std::stringstream ss;
char sz[18];




void CmdStrCallback(const std_msgs::String::ConstPtr& msg);
void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg);
void NavInfo_Callback(const classifier_hsr::CBA_NavInfo::ConstPtr& msg);
void NavInfo_Callback(const classifier_hsr::CBA_NavInfo::ConstPtr& msg)
{
  int width = msg->width;
  int height = msg->height;  

  std::vector<int> cell_occupancy_type(msg->cell_occupancy_type.size(),0);
  std::vector<int> action_policy_type(msg->action_policy_type.size(),0);
  std::vector<int> robot_local_cell_indices(msg->robot_local_cell_indices.size(),0);
  std::vector<int> state_type(msg->state_type.size(),0);
  std::vector<float> state_Distance(msg->state_distance.size(),0);
  std::vector<float> NearestHumanVector(2,0.0);
  std::vector<float> RobotHeadingDirection(2,0.0);

  for(int i = 0; i < msg->cell_occupancy_type.size(); i++)
    cell_occupancy_type[i]=msg->cell_occupancy_type[i];

  for(int i = 0; i < msg->action_policy_type.size(); i++)
    action_policy_type[i]=msg->action_policy_type[i];

  for(int i = 0; i < msg->robot_local_cell_indices.size(); i++)
    robot_local_cell_indices[i]=msg->robot_local_cell_indices[i];

  for(int i = 0; i < msg->state_type.size(); i++)
    state_type[i]=msg->state_type[i];

  for(int i = 0; i < msg->state_distance.size(); i++)
    state_Distance[i]=msg->state_distance[i];

   NearestHumanVector[0]=msg->unit_x_to_human;
   NearestHumanVector[1]=msg->unit_y_to_human;

 RobotHeadingDirection[0]=msg->unit_base_link_x;
 RobotHeadingDirection[1]=msg->unit_base_link_x;


// std::cout<<"NH x:" << NearestHumanVector[0]<< ",  y : "<<NearestHumanVector[1]<<endl;  

// std::cout<<"Heading x:" << RobotHeadingDirection[0]<< ",  y : "<<RobotHeadingDirection[1]<<endl;  

  // std::cout << "I got some message from Kinect" << std::endl;
  // std::cout << msg->cell_occupancy_type.size() << std::endl;
  //std::cout << msg->action_policy_type.size() << std::endl;

  //Save to MapParam

  CurrentMap.setWidth(width);
  CurrentMap.setHeight(height);
  CurrentMap.set_Cell_Info(cell_occupancy_type);
  CurrentMap.set_OCC_Info(action_policy_type);
  CurrentMap.set_Robot_Info(robot_local_cell_indices);
  CurrentMap.set_State_Type(state_type);
  CurrentMap.set_State_Distance(state_Distance);
  CurrentMap.set_NearestHuman_V(NearestHumanVector);
  CurrentMap.set_RobotHeading_V(RobotHeadingDirection);

 //printf("I am here\n");

  vector<float> FeatureVector = m_Manager.getFeaturevector();
  int AutoDesiredaction=m_Manager.getDirectionfromCBA(FeatureVector);


  if(m_Manager.boolAuto){
    printf("Auto Mode\n");
    ros::Rate r(0.45);
    
     //   try {
     //     libsocket::inet_stream sock(host,port,LIBSOCKET_IPv4);
     //      sock >> answer;
     //      std::cout << answer;
          
     //    // ss<<1;
     //    // sender=ss.str();

     //    //ithcmd=cmdarry[i];
     //     //ithcmd=CMD_GUI;

     //    printf("Automode command : %d",AutoDesiredaction);
     //    sprintf(sz,"%d",AutoDesiredaction);
     //    sock<<sz;
        
     //    //boost::this_thread::sleep( boost::posix_time::seconds(1));

        

     //  } catch (const libsocket::socket_exception& exc){
     //     std::cerr << exc.mesg;
     // }

     r.sleep();
 
  }
}





void Goal_Matlab_Callback(const geometry_msgs::Pose2D::ConstPtr& msg);
void Goal_Matlab_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{ 
   vector<int> scaledGoalPos(2,0);
   scaledGoalPos[0]=static_cast<int>(msg->x);
   scaledGoalPos[1]=static_cast<int>(msg->y);


   m_Manager.m_Goal[0]=m_Manager.Local_X_start+scaledGoalPos[0]*m_Manager.Scale_constant-1;
   m_Manager.m_Goal[1]=m_Manager.Local_Y_start+scaledGoalPos[1]*m_Manager.Scale_constant-1;

   cout<<"Goal Pose is x: "<<m_Manager.m_Goal[0]<<"y : "<<m_Manager.m_Goal[1]<<endl;

}


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
  // ros::Rate r(5);

  m_Manager.pMapParam=&CurrentMap;
  m_Manager.pClassifier=&elmclassifier_;
  //m_Manager.LoadDataFile();


 	// std::cout <<"Helloworld" << std::endl; 
   ros::Publisher  Matlab_Pub;
   ros::Publisher  Map_Pub;
   ros::Subscriber cmd_sub;
   ros::Subscriber cmd_sub2;
   ros::Subscriber grid_sub;   
   ros::Subscriber grid_sub2; 
   ros::Subscriber matlabgoal_sub; 
   ros::NodeHandle n;

 //  ros::Timer timer1 = n.createTimer(ros::Duration(0.5), callback1);
 //  Publisher
   //Subscriber
   cmd_sub  = n.subscribe<std_msgs::String>("/CBA_cmd_str", 50,CmdStrCallback); 
   cmd_sub2 = n.subscribe<std_msgs::Int8>("/CBA_cmd_int", 50,CmdIntCallback);
   grid_sub = n.subscribe<classifier_hsr::CBA_NavInfo>("/CBA_grid_occ_topic", 20, NavInfo_Callback);  
   matlabgoal_sub= n.subscribe<geometry_msgs::Pose2D>("/Matlab_goal", 10, Goal_Matlab_Callback);
   ros::Rate loop_rate(20);

  while (ros::ok())
  {

		 ros::spinOnce();
   //  m_Manager.PublishMapInfo2Matlab(Matlab_Pub);
      loop_rate.sleep();   

      // ros::spin();
   }
  // ros::spin();
  return 0;
}

void CmdStrCallback(const std_msgs::String::ConstPtr& msg)          
{
    //ROS_INFO("str msg");
    ROS_INFO("Cur cmd :%s ",(msg->data).c_str());


    return;
}

void Send2Trikey(int cmd)
{


}


void CmdIntCallback(const std_msgs::Int8::ConstPtr& msg)
{
    //ROS_INFO("int msg");
    ROS_INFO("Cur cmd :%d ",msg->data);

    int cmdfromGUI=(int)(msg->data);
   
    m_Manager.ActionfromGUICmd(cmdfromGUI);

  
        return;
}

