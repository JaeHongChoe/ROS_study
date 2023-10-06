#!/usr/bin/env python

import rospy, rospkg
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from xycar_motor.msg import xycar_motor

global pub
global msg_joint_states
global l_wheel, r_wheel

def callback(data):
     
   global msg_joint_states, l_wheel, r_wheel, pub
   Angle = data.angle
   msg_joint_states.header.stamp = rospy.Time.now()
   steering = math.radians(Angle * 0.4)
      
   if l_wheel >= 3.14:
      l_wheel = -3.14
      r_wheel = -3.14
   else:
      l_wheel += 0.01
      r_wheel += 0.01

   msg_joint_states.position = [steering, steering, r_wheel, l_wheel, r_wheel, l_wheel]

   pub.publish(msg_joint_states)

def start():

	global msg_joint_states, l_wheel, r_wheel, pub

	rospy.init_node('converter')
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)


	msg_joint_states = JointState() # hello_xycar
	msg_joint_states.header = Header()
	msg_joint_states.name = ['front_right_hinge_joint', 'front_left_hinge_joint', 
		      'front_right_wheel_joint', 'front_left_wheel_joint',
		       'rear_right_wheel_joint','rear_left_wheel_joint']
	msg_joint_states.velocity = []
	msg_joint_states.effort = [] 

	l_wheel = -3.14
	r_wheel = -3.14

	rospy.Subscriber("xycar_motor", xycar_motor, callback)
	rospy.spin()

if __name__ == '__main__':
   start()


