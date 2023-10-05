#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

name = "sender"
pub_topic = "my_topic"

rospy.init_node(name)
pub = rospy.Publisher(pub_topic, Int32, queue_size=1)

rate = rospy.Rate(1000)
count = 1

while(pub.get_num_connections() ==0):
   count = 1


while not rospy.is_shutdown():
   pub.publish(count)
   count +=1
   rate.sleep()
