#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import termios, fcntl, sys, os
import rospy

#import service library
from std_srvs.srv import Empty

#topic to command
# Twist better than odometry TODO ask why
twist_topic="/uwsim/g500_A/twist_command"
joint_topic="/uwsim/g500_A/joint_command"
#base velocity for the teleoperation
baseVelocity=0.01

#Console input variables to teleop it from the console
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

##create the publishers
pubTwist = rospy.Publisher(twist_topic, TwistStamped,queue_size=1)
rospy.init_node('keyboardCommandVeh_A')
msgTwist = TwistStamped()
modality = True # true: at each while velocitty is resetted

#The try is necessary for the console input!
try:
    while not rospy.is_shutdown():

        if (modality):
            msgTwist = TwistStamped()

        try:
            c = sys.stdin.read(1)
            ##Depending on the character set the proper speeds
            if c=='\x1b':  ##This means we are pressing an arrow!
                c2= sys.stdin.read(1)
                c2= sys.stdin.read(1)
                if c2=='A':
                    msgTwist.twist.angular.y=-baseVelocity
                elif c2=='B':
                    msgTwist.twist.angular.y=baseVelocity
                elif c2 == 'C':
                    msgTwist.twist.angular.z=baseVelocity
                elif c2 == 'D':
                    msgTwist.twist.angular.z=-baseVelocity
            elif c == 'w':
                msgTwist.twist.linear.x = baseVelocity
            elif c == 's':
                msgTwist.twist.linear.x = -baseVelocity
            elif c == 'd':
                msgTwist.twist.linear.y = baseVelocity
            elif c == 'a':
                msgTwist.twist.linear.y = -baseVelocity
            elif c == 'v':
                msgTwist.twist.linear.z = baseVelocity
            elif c == 'f':
                msgTwist.twist.linear.z = -baseVelocity
            elif c == 'q':
                msgTwist.twist.angular.x = -baseVelocity
            elif c == 'e':
                msgTwist.twist.angular.x = baseVelocity      
            #Increase Velocity send command
            elif c == 'y':
                baseVelocity+=0.1;
                print "baseVelocity: " , baseVelocity
            elif c == 'x':
                if baseVelocity > 0.2:
                    baseVelocity-=0.1
                elif baseVelocity > 0:
                    baseVelocity -= 0.01
                else:
                    baseVelocity = 0
                print "baseVelocity: " , baseVelocity
            else:
                print 'wrong key pressed, and reset all vehicle velocities'
                msgTwist = TwistStamped()
                modality = not modality

            while c!='':
                c = sys.stdin.read(1)
        except IOError: pass

        ##publish the message
        print 'velocity vehicle published:'
        print (msgTwist.twist.linear.x,  msgTwist.twist.linear.y, msgTwist.twist.linear.z)
        print (msgTwist.twist.angular.x,  msgTwist.twist.angular.y,  msgTwist.twist.angular.z )
        pubTwist.publish(msgTwist)
        rospy.sleep(0.1)

##Other input stuff
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
