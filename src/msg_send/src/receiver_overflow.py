#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

name = "receiver"
sub_topic = "my_topic"

def callback(msg):
   rospy.loginfo("callback is being processed")
   rospy.sleep(5)
   print msg.data


rospy.init_node(name)
rospy.Subscriber(sub_topic, Int32, callback, queue_size=10000)
rospy.spin()
