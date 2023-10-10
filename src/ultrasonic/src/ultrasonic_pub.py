#!/usr/bin/env python

import serial, time, rospy, re
from std_msgs.msg import Int32

ser_front = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    )

def read_sensor():
    serial_data = ser_front.readline()
    ser_front.flushInput()
    ser_front.flushOutput() 
    ultrasonic_data = int(filter(str.isdigit, serial_data))
    msg.data = ultrasonic_data
  
if __name__ == '__main__':

    rospy.init_node('ultrasonic_pub', anonymous=False) # initialize node
    pub = rospy.Publisher('ultrasonic', Int32, queue_size=1)

    msg = Int32() # message type
    while not rospy.is_shutdown():
        read_sensor() 
        pub.publish(msg) # publish a message
        time.sleep(0.2)
    
    ser_front.close()


