#!/usr/bin/env python

import rospy
import time
from xycar_motor.msg import xycar_motor

motor_c = xycar_motor()

rospy.init_node('driver')

pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

def motor_pub(angle,speed):
   global pub
   global motor_c

   motor_c.angle = angle
   motor_c.speed = speed

   pub.publish(motor_c)

speed = 3

while not rospy.is_shutdown():
   angle = -50
   for i in range(70):
      motor_pub(angle, speed)
      time.sleep(0.1)

   angle = 0
   for i in range(30):
      motor_pub(angle, speed)
      time.sleep(0.1)

   angle = 50
   for i in range(70):
      motor_pub(angle, speed)
      time.sleep(0.1)

   angle = 0 
   for i in range(30):
      motor_pub(angle, speed)
      time.sleep(0.1)
