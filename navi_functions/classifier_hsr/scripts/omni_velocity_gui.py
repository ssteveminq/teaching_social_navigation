#!/usr/bin/env python
import roslib
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


class BaseMoveCBA(object):
	def __init__(self, wait=0.0):
		# initialize action client
		self.vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
		rospy.Subscriber("/CBA_cmd_int", Int8, self.intcallback)
		self.tw = geometry_msgs.msg.Twist()

		while self.vel_pub.get_num_connections() == 0:
    			rospy.sleep(0.1)


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
	    
	       # command = data.data
		if data.data == 1:
			self.tw.angular.z = 0.8
		elif data.data == 2:
		    self.tw.linear.x = 1.0
		elif data.data == 3:
		    self.tw.angular.z = -0.8
		# elif data.data == 3:
	    # 	self.p.positions = [0, 0, 0]
	    # 	self.p.velocities = [0, 0, 1.0]
		else:
			self.tw.angular.z = 0.0
		
		self.vel_pub.publish(self.tw)



# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# cli.wait_for_result()
if __name__ == '__main__':
    rospy.init_node('test')
    CBA_GUI_BASE = BaseMoveCBA(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    CBA_GUI_BASE.listener()