#!/usr/bin/env python

import rospy
from std_msgs.msg import String

name = "receiver"
sub_topic = "long_string"

def callback(data):
   current_time = str(rospy.get_time())
   arrival_data = str(data.data).split(".")

   time_diff = float(current_time) - float(arrival_data[1])
   string_size = len(arrival_data[0])
   rospy.loginfo(str(string_size) + "byte:" + str(time_diff) +" second")
   rospy.loginfo("speed:" + str(float(string_size)/time_diff) + "byte/s")


rospy.init_node(name, anonymous=True)
rospy.loginfo("Init")
rospy.Subscriber(sub_topic, String, callback)
rospy.spin()
