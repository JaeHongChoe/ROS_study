#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

rospy.init_node('teacher')
pub = rospy.Publisher('msg_to_student', Int32)
rate = rospy.Rate(2)
count = 1

while not rospy.is_shutdown():
  pub.publish(count)
  count +=1
  rate.sleep()

