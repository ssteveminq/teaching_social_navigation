#!/usr/bin/env python
import roslib
import math
import sys
import rospy
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
import geometry_msgs.msg
import controller_manager_msgs.srv
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry


class BaseMoveCBA(object):
    def __init__(self, wait=0.0):
        # initialize action client
        self.cli = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        # publisher for delvering command for base move
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        # Subscriber for receiving GUI command 
        rospy.Subscriber("/CBA_action_cmd", Int8, self.intcallback)
        rospy.Subscriber("/hsrb/odom", Odometry, self.odometryInput)
        # message initialization
        self.tw = geometry_msgs.msg.Twist()
        #
        while self.vel_pub.get_num_connections() == 0:
                rospy.sleep(0.1)
        # Initialization of goal & trajectory
        self.goal = control_msgs.msg.FollowJointTrajectoryGoal()
        self.traj = trajectory_msgs.msg.JointTrajectory()
        self.traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        self.p = trajectory_msgs.msg.JointTrajectoryPoint()
        self.p.positions = [1, 0, 0.8]
        self.p.velocities = [0, 0, 0]
        self.p.time_from_start = rospy.Time(5)
        self.traj.points = [self.p]
        self.goal.trajectory = self.traj
        self.cli.wait_for_server()
        self.robot_pos_x=0.0
        self.robot_pos_y=0.0
        self.robot_pos_z=0.0
        self.robot_ori_z=0.0

    def odometryInput(self, data):
        self.robot_pos_x = data.pose.pose.position.x
        self.robot_pos_y = data.pose.pose.position.y
        self.robot_pos_z = data.pose.pose.position.z
        self.robot_ori_z = data.pose.pose.orientation.z
        self.robot_ori_z = math.asin(self.robot_ori_z)*2
        # rospy.loginfo(rospy.get_caller_id()+"x : %.3lf , y : %.3lf , z : %.3lf",self.robot_pos_x,self.robot_pos_y,self.robot_ori_z)


    def listener(self,wait=0.0):
        
        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        self.list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        running = False
        while running == False:
            rospy.sleep(0.1)
            for c in self.list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True

        rospy.spin()            
        # fill ROS message
    def intcallback(self, data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %d",data.data)
        self.pre_ori_z=self.robot_ori_z
         # command = data.data
        if data.data == 1:                       #Going home
            self.tw.linear.x =0
            self.tw.angular.z = 0.5
            self.vel_pub.publish(self.tw)
            # self.p.positions = [self.robot_pos_x,self.robot_pos_y,self.robot_ori_z+math.pi/4]
            # self.p.velocities =[0,0,0]
            # self.traj.points = [self.p]
            # self.goal.trajectory = self.traj
            # self.p.time_from_start = rospy.Time(3)
            # self.cli.send_goal(self.goal)
            # self.cli.wait_for_result()
        elif data.data == 2:
            # self.p.positions = [self.robot_pos_x+math.cos(self.pre_ori_z),self.robot_pos_y+math.sin(self.pre_ori_z),self.pre_ori_z]
            # self.p.velocities =[0,0,0]
            # self.traj.points = [self.p]
            # self.goal.trajectory = self.traj
            # self.p.time_from_start = rospy.Time(3)
            # self.cli.send_goal(self.goal)
            # self.cli.wait_for_result()
            self.tw.linear.x = 5.5
            self.vel_pub.publish(self.tw)
        elif data.data == 3:
            # self.p.positions = [self.robot_pos_x,self.robot_pos_y,self.robot_ori_z-math.pi/4]
            # self.traj.points = [self.p]
            # self.goal.trajectory = self.traj
            # self.p.time_from_start = rospy.Time(3)
            # self.cli.send_goal(self.goal)
            # self.cli.wait_for_result()
            self.tw.linear.x =0
            self.tw.angular.z = -0.5
            self.vel_pub.publish(self.tw)
        elif data.data == 4:                     #Going home
            self.p.positions = [0,0,0.0]
            self.p.velocities =[0,0,0]
            self.traj.points = [self.p]
            self.goal.trajectory = self.traj
            self.p.time_from_start = rospy.Time(3)
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()
        elif data.data == 5:
            self.p.positions = [0, 1, 0.0]
            self.p.velocities = [0, 0, 0]
            self.traj.points = [self.p]
        elif data.data == 10:
            self.goal.trajectory = self.traj
            self.p.time_from_start = rospy.Time(3)
            self.cli.send_goal(self.goal)
            self.cli.wait_for_result()
        # elif data.data == 3:
        #   self.p.positions = [0, 0, 0]
        #   self.p.velocities = [0, 0, 1.0]
        else:
            self.tw.angular.z = 0.0
        
        



# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# cli.wait_for_result()
if __name__ == '__main__':
    rospy.init_node('test')
    CBA_GUI_BASE = BaseMoveCBA(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    CBA_GUI_BASE.listener()