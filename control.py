#!/usr/bin/env python

import os
import rospy
from race.msg import drive_param
from race.msg import pid_input

path_to_watch = os.path.expanduser('~/circle.txt')
cache = set()

kp = 14.0
kd = 0.09
servo_offset = 18.5  # zero correction offset in case servo is misaligned.
prev_error = 0.0
vel_input = 10.0

# Update the variable below to change distance from wall
# Determine what offset relates to what distance in feet experimentally
OFFSET = 0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


def control(data):
    global prev_error
    global vel_input
    global kp
    global kd
    global OFFSET
    global before

    prev_offset = OFFSET
    orig_vel_input = vel_input
    if os.path.isfile(path_to_watch):
        with open(path_to_watch, 'r') as content_file:
            content = content_file.readlines()
            for line in content:
		print "\n\nBEGINNING NEW PROCESSING OF TEXT FILE \n\n"
                if line not in cache:
                    cache.add(line)
                    print "Should start to turn! "+str(line)

                    # we will actually parse this line in the future
                    # every line has to be unique. So must follow format "# - command" per line

                    # spike the offset to make a turn
                    OFFSET = 2.0
		    vel_input = vel_input/5.0
    else:
        print "~/circle.txt does not exist"

    data.pid_error = data.pid_error + OFFSET
    angle = data.pid_error*kp
    if angle > 100:
	angle = 100
    if angle < -100:
	angle = -100

    msg = drive_param()
    if data.pid_vel == 0:
	msg.velocity = vel_input
	#msg.velocity = -8
    else:
	msg.velocity = vel_input

    msg.angle = angle
    print "PUBLISHING MESSAGE: "+str(msg)
    pub.publish(msg)

    # reset the offset to the original value
    OFFSET = prev_offset
    vel_input = orig_vel_input

def populate_cache():
    if os.path.isfile(path_to_watch):
        with open(path_to_watch, 'r') as content_file:
            content = content_file.readlines()
            for line in content:
                if line not in cache:
                    cache.add(line)
                    print line
    else:
        print "~/circle.txt does not exist"


if __name__ == '__main__':
    global kp
    global kd
    global vel_input
    global OFFSET
    print("Control node initialized. Listening for error input")
    kp = input("Enter Kp Value: ")
    kd = input("Enter Kd Value: ")
    vel_input = input("Enter Velocity: ")
    OFFSET = input("Enter Offset: ")
    populate_cache()
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", pid_input, control)
    rospy.spin()

