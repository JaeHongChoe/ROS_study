#!/usr/bin/env python
import rospy
from std_msgs.msg import String
name = "sender"
pub_topic = "long_string"

rospy.init_node(name, anonymous=True)
pub = rospy.Publisher(pub_topic, String, queue_size=1)

hello_str = String()
rate = rospy.Rate(1)

#pub_size = 1000000
#pub size = 20000000 # 20M byte
#pub size = 30000000 # 30M byte
#pub size = 40000000 # 40M byte
pub_size = 50000000 # 50M byte


my_string = ""
for i in range(pub_size): 
   my_string += "#"


while not rospy.is_shutdown():
   hello_str.data = my_string + ":" + str(rospy.get_time())
   pub.publish(hello_str)
# rospy.loginfo(str(hello_str.data).split(":")[1])
   rate.sleep()
