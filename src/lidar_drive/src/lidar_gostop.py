#!/usr/bin/env python

# import msg file
import rospy, time
from sensor_msgs.msg import LaserScan 
from xycar_msgs.msg import xycar_motor 


motor_msg = xycar_motor()
distance = [] # prepare storage to save the distance value for lidar

# if topic about lidar enter, define the callback function to implement
def callback(data):
    global distance, motor_msg
    distance = data.ranges

# define function for going
def drive_go():
    global motor_msg
    motor_msg.speed = 5
    motor_msg.angle = 0
    pub.publish(motor_msg)

# define function for stoping
def drive_stop():
    global motor_msg
    motor_msg.speed = 0
    motor_msg.angle = 0
    pub.publish(motor_msg)

# make node and define sub and pub node
rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

# ready to connect lidar
time.sleep(3)

# scan from 60 degree to 120 degree, because front side is 90 degree
while not rospy.is_shutdown():
    ok = 0
    for degree in range(60, 120):
        if distance[degree] <= 0.3:
            ok += 1
        if ok > 3:          # if more than three are lower 30cm, do stop
            drive_stop()
            break
    if ok <= 3:             # else go
        drive_go()
