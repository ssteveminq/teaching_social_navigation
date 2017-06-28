#include "navi_service_node.h"


villa_navi_srv::villa_navi_srv():index(0),m_numofhuman(0),m_receiveiter(0){
	}
villa_navi_srv::~villa_navi_srv(){}


void villa_navi_srv::Publish_nav_target(float _x, float _y, float _theta)
{
	
	
		move_base_msgs::MoveBaseActionGoal Navmsgs;
		Navmsgs.header.stamp =  ros::Time::now();
		Navmsgs.goal.target_pose.header.frame_id = "base_link";

		// geometry_msgs::Vector3Stamped gV, tV;

	 //    gV.vector.x = leg_target[0];
	 //    gV.vector.y = leg_target[1];
	 //    gV.vector.z = 1.0;

	 //    // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
	 //    gV.header.stamp = ros::Time();
	 //    gV.header.frame_id = "base_range_sensor_link";
	 //    listener.transformVector("/map", gV, tV);

	 //    std::vector<double> tempVec(2,0.0);
	 //    tempVec[0]=tV.vector.x;
		// tempVec[1]=tV.vector.y;


		 Navmsgs.goal.target_pose.pose.position.x=_x;
		 Navmsgs.goal.target_pose.pose.position.y=_y;
		 Navmsgs.goal.target_pose.pose.position.z=0.5;

		 Navmsgs.goal.target_pose.pose.orientation.x=0.0;
		 Navmsgs.goal.target_pose.pose.orientation.y=0.0;
		 Navmsgs.goal.target_pose.pose.orientation.z=0.0;
		 Navmsgs.goal.target_pose.pose.orientation.w=1.0;

		 // setViewpointTarget(leg_target);
		 setNavTarget_pub.publish(Navmsgs);
		 ROS_INFO("navgation published");
}


bool villa_navi_srv::goTarget(villa_navi_service::GoTargetPos_rel::Request &req, villa_navi_service::GoTargetPos_rel::Response &res)
{


	// std::vector<float> targetpos(3,0.0);
	// targetpos[0]=_x;
	// targetpos[1]=_y;
	// targetpos[2]=_theta;
	if(req.x_from_baselink==NULL)
	{
		Publish_nav_target(0.5,0.0,0.0);
		res.is_possible_go=true;
	}
	else
	{

	
	Publish_nav_target(req.x_from_baselink,req.y_from_baselink,req.theta_from_baselink);
	res.is_possible_go=true;
	}


}


void villa_navi_srv::setViewpointTarget(const std::vector<double> pos)
{

	geometry_msgs::Point GazePoint_msg;


	if(pos[0]==0.0 && pos[1]==0.0)
	{
		GazePoint_msg.x=2.0;
		GazePoint_msg.y=0.0;	
	}
	else
	{
	
	GazePoint_msg.x=pos[0];
	GazePoint_msg.y=pos[1];
	
	}

	GazePoint_msg.z=1.0;

	Gaze_point_pub.publish(GazePoint_msg);

	std_msgs::Bool activateGaze_msg;
	activateGaze_msg.data=true;

	Gaze_activate_pub.publish(activateGaze_msg);

	// ros::Duration(0.5).sleep();


}











