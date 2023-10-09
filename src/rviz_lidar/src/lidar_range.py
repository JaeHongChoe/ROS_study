#!/usr/bin/env python
import serial, time, rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header

rospy.init_node('lidar_range')

pub1 = rospy.Publisher('scan1', Range, queue_size=1)
pub2 = rospy.Publisher('scan2', Range, queue_size=1)
pub3 = rospy.Publisher('scan3', Range, queue_size=1)
pub4 = rospy.Publisher('scan4', Range, queue_size=1)

msg = Range()
h = Header()
h.frame_id = "sensorXY"
msg.header = h
msg.radiation_type = Range().ULTRASOUND
msg.min_range = 0.02
msg.max_range = 2.0
msg.field_of_view = (30.0/180.0)*3.14

while not rospy.is_shutdown():
   msg.header.stamp = rospy.Time.now()

   msg.range = 0.4
   pub1.publish(msg)

   msg.range = 0.8
   pub2.publish(msg)

   msg.range = 1.2
   pub3.publish(msg)

   msg.range = 1.6
   pub4.publish(msg)

   time.sleep(0.2)

