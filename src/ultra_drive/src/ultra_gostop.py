#!/usr/bin/env python

# ultra_gostop.py

# import msg file
import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor 


motor_msg = xycar_motor()
ultra_msg = None # prepare storage to save the distance value for ultrasonic

# if topic about ultra enter, define the callback function to implement
def callback(data):
    global ultra_msg
    ultra_msg = data.data

# define function for going
def drive_go():
    global motor_msg, pub
    motor_msg.speed = 5
    motor_msg.angle = 0
    pub.publish(motor_msg)

# define function for stoping
def drive_stop():
    global motor_msg, pub
    motor_msg.speed = 0
    motor_msg.angle = 0
    pub.publish(motor_msg)

# make node and define sub and pub node
rospy.init_node('ultra_driver')
rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray, callback, queue_size = 1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

# ready to connect ultra
time.sleep(2)

# scan from 60 degree to 120 degree, because front side is 90 degree
while not rospy.is_shutdown():
    if ultra_msg[2] > 0 and ultra_msg[2] < 10:
        drive_stop()  # if front ultrasonic sensor the detected distance information is 
                      # 0 < distance < 10cm, do stop
    else:             # other situation is infinition, so then there are no obstacles.
        drive_go()    # therefore, let's go
