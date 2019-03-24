#!/usr/bin/env python

from nav_msgs.msg import Odometry
import termios, fcntl, sys, os
import rospy

#import service library
from std_srvs.srv import Empty

#topic to command
# Twist better than odometry? TODO ask why
twist_topic="/uwsim/g500_A/dataNavigator_A"
#base velocity for the teleoperation (0.5 m/s) / (0.5rad/s)
baseVelocity=0.5

#Console input variables to teleop it from the console
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

##create the publisher
pub = rospy.Publisher(twist_topic, Odometry,queue_size=1)
rospy.init_node('keyboardCommandOdom_A')

#The try is necessary for the console input!
try:
    while not rospy.is_shutdown():
        msg = Odometry()
        msg.pose.pose.position.x=0.0
        msg.pose.pose.position.y=0.0
        msg.pose.pose.position.z=0.0
        msg.pose.pose.orientation.x=0.0
        msg.pose.pose.orientation.y=0.0
        msg.pose.pose.orientation.z=0.0
        msg.pose.pose.orientation.w=1
        
        try:
            c = sys.stdin.read(1)
            ##Depending on the character set the proper speeds
            if c=='\x1b':  ##This means we are pressing an arrow!
                c2= sys.stdin.read(1)
                c2= sys.stdin.read(1)
                if c2=='A':
                    msg.twist.twist.angular.y=-baseVelocity
                elif c2=='B':
                    msg.twist.twist.angular.y=baseVelocity
                elif c2 == 'C':
                    msg.twist.twist.angular.z=baseVelocity
                elif c2 == 'D':
                    msg.twist.twist.angular.z=-baseVelocity
            elif c == 'w':
                msg.twist.twist.linear.x = baseVelocity
            elif c == 's':
                msg.twist.twist.linear.x = -baseVelocity
            elif c == 'd':
                msg.twist.twist.linear.y = baseVelocity
            elif c == 'a':
                msg.twist.twist.linear.y = -baseVelocity
            elif c == 'v':
                msg.twist.twist.linear.z = baseVelocity
            elif c == 'f':
                msg.twist.twist.linear.z = -baseVelocity
            else:
                print 'wrong key pressed'
            while c!='':
                c = sys.stdin.read(1)
        except IOError: pass

        ##publish the message
        pub.publish(msg)
        rospy.sleep(0.1)

##Other input stuff
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
