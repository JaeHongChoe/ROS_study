#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(msg):
  print msg.data

rospy.init_node('student')

sub = rospy.Subscriber('msg_to_student',String, callback)

rospy.spin()

