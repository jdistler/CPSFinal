#!/usr/bin/env python

import os
import time
import rospy
from race.msg import drive_param
from race.msg import pid_input

path_to_watch = os.path.expanduser('~/circle.txt')
cache = set()

kp = 250
kd = 0
vel_input = 17

# Update the variable below to change distance from wall
# 1 foot offset is 0.16, bigger is closer to wall
OFFSET = 0.35

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


def control(data):
    global vel_input
    global kp
    global kd
    global OFFSET

    circle_detected = False

    if os.path.isfile(path_to_watch):
        with open(path_to_watch, 'r') as content_file:
            content = content_file.readlines()
            for line in content:
                print "\n\nBEGINNING NEW PROCESSING OF TEXT FILE \n\n"
                if line not in cache:
                    cache.add(line)
                    print "Should start to turn! " + str(line)

                    # we will actually parse this line in the future
                    # every line has to be unique. So must follow format "# - command" per line

                    circle_detected = True

    else:
        print "~/circle.txt does not exist"

    if circle_detected:
        print "Circle Detected"
        # ignore any of the wall distance correction logic, just set angle, velocity and publish
        angle = -170
        msg = drive_param()
        msg.velocity = vel_input
        msg.angle = angle
        print "PUBLISHING MESSAGE: " + str(msg)
        pub.publish(msg)
	time.sleep(1)
    else:
        print "Circle Not Detected"
        data.pid_error = data.pid_error + OFFSET
        angle = data.pid_error * kp
        if angle > 100:
            angle = 100
        if angle < -100:
            angle = -100

        msg = drive_param()
        if data.pid_vel == 0:
            msg.velocity = -8
	    angle = 0
        else:
            msg.velocity = vel_input

        msg.angle = angle
        print "PUBLISHING MESSAGE: " + str(msg)
        pub.publish(msg)


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
    # kp = input("Enter Kp Value: ")
    # kd = input("Enter Kd Value: ")
    # vel_input = input("Enter Velocity: ")
    # OFFSET = input("Enter Offset: ")
    populate_cache()
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", pid_input, control)
    rospy.spin()
