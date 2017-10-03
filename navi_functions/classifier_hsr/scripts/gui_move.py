#!/usr/bin/env python

import rospy
import sys
import rospkg
import yaml
from std_msgs.msg import String
from std_msgs.msg import Int8
from PyQt4 import QtGui, QtCore

class SpeechGui(QtGui.QWidget):

  def __init__(self):
      QtGui.QWidget.__init__(self)
 
      newFont = QtGui.QFont("Times", 24, QtGui.QFont.Bold)

      # Add a main layout
      mainLayout = QtGui.QVBoxLayout(self)
      #mainLayout->setMeubBar(menuBar)
      # Add buttons with the commands
      grid = QtGui.QGridLayout()
      grid.setSpacing(20)

      # Initialize rosnode
      rospy.init_node("Move_gui")
   
      # Default values for speech listeners  
      rospack = rospkg.RosPack()
      default_pub_topic = 'action_commands'

      # # Pull values from rosparam
       # # Wait for listener to be ready to know what commands to send
      # self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM, None)
      # rospy.loginfo("Waiting for speech service for keywords")
      # rospy.wait_for_service(self.service_topic)
      # rospy.loginfo("Speech service loaded")

      # Get commands from the listener
      self.commands = ["Turn Left", "Turn Right", "Go Forward", "Go Backward", "Rotate_CW", "Rotate_CCW", "Continue GO", "Continue Back"] 
      # self.commands.sort()
 
      positions = [(i,j) for i in range(len(self.commands)) for j in range(4)]
           
      for position, name in zip(positions, self.commands):
          button = QtGui.QPushButton(name)
          button.setObjectName('%s' % name)
          button.setFont(newFont)
          button.setStyleSheet("background-color: #ccffe6")
          button.clicked.connect(self.handleButton)
          grid.addWidget(button, *position)

      mainLayout.addLayout(grid)
      mainLayout.addStretch()
      
      # Show the GUI 
      self.adjustSize()
      self.setWindowTitle("BASE MOVE GUI")
      self.move(400,400)
      self.show()
      self.raise_()

      # # Create the publisher to publish the commands to
      # self.pub = rospy.Publisher("CBA_cmd_str", String, queue_size=1)
      self.pub2 = rospy.Publisher("nav_cmd_int", Int8, queue_size=1)
      rospy.loginfo("Finished initializing BASE GUI")

  # Button handler after its clicked
  def handleButton(self):
      clicked_button = self.sender()
      #print "you're here!"
      # # Publish everytime a command is selected from the combo box
      command = str(clicked_button.objectName())
      if command =="East": 
        num_cmd=1
      elif command =="E-N":
        num_cmd=2
      elif command =="North":
        num_cmd=3
      elif command =="Go Backward":
        num_cmd=4
      elif command =="Rotate_CW":
        num_cmd=5
      elif command =="Rotate_CCW":
        num_cmd=6
      elif command =="Continue GO":
        num_cmd=7
      elif command =="Continue Back":         
        num_cmd=9
      else:
        num_cmd=0
    #  print command
     # print num_cmd

     #if self.str_msg == 'String':
      msg = String()
      msg.data = command

      msg2= Int8()
      msg2.data=num_cmd

      # self.pub.publish(msg)
      self.pub2.publish(msg2)
     #  self.pub2.publish(msg2)
      # else:
       #  keyphrase = StampedString()
       #  keyphrase.keyphrase = command
       #  keyphrase.stamp = rospy.get_rostime()
       #  self.pub.publish(keyphrase)

def gui_start():
    app = QtGui.QApplication(sys.argv)
    sg = SpeechGui()
    sys.exit(app.exec_())


gui_start()
