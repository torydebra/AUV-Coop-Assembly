#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from freefloating_gazebo.srv import ControlType
import termios, fcntl, sys, os
import rospy

#import service library
from std_srvs.srv import Empty

#topic to command
twist_topic="/g500_A/body_velocity_setpoint"
joint_topic="/g500_A/joint_setpoint"
#service to set control mode
bodyServ = '/g500_A/controllers/body_velocity_control'
jointServ = '/g500_A/controllers/joints_velocity_control'
#base velocity for the teleoperation
baseVelocity=0.8
baseJoint=0.1

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
pubJoint = rospy.Publisher(joint_topic, JointState,queue_size=1)
rospy.init_node('keyboardCommand_A')

#set control mode to velocity
rospy.wait_for_service(bodyServ)
rospy.wait_for_service(jointServ)

try:
    str = ["x", "y", "z", "pitch", "yaw"]
    body_vel_control = rospy.ServiceProxy(bodyServ, ControlType)
    body_vel_control(str)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e
try:
    str = ["Slew", "Shoulder", "Elbow", "JawRotate", "JawOpening", "JawOpening2"]
    joint_vel_control = rospy.ServiceProxy(jointServ, ControlType)
    joint_vel_control(str)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e


#The try is necessary for the console input!
try:
    while not rospy.is_shutdown():
        msgTwist = TwistStamped()
        msgJoint = JointState()
        msgTwist.twist.linear.x = 0
        msgTwist.twist.linear.y = 0
        msgTwist.twist.linear.z = 0
        msgTwist.twist.angular.x = 0
        msgTwist.twist.angular.x = 0
        msgTwist.twist.angular.z = 0
        pubTwist.publish(msgTwist)

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
            #JointsCommands
            elif c == 't':
            	msgJoint.name.append("Slew")
            	msgJoint.velocity.append(baseJoint)
            elif c == 'z':
            	msgJoint.name.append("Shoulder")
            	msgJoint.velocity.append(baseJoint)
            elif c == 'u':
            	msgJoint.name.append("Elbow")
            	msgJoint.velocity.append(baseJoint)
            elif c == 'i':
            	msgJoint.name.append("JawRotate")
            	msgJoint.velocity.append(baseJoint)
            elif c == 'o':
            	msgJoint.name.append("JawOpening")
            	msgJoint.velocity.append(baseJoint)
            elif c == 'g':
            	msgJoint.name.append("Slew")
            	msgJoint.velocity.append(-baseJoint)
            elif c == 'h':
            	msgJoint.name.append("Shoulder")
            	msgJoint.velocity.append(-baseJoint)
            elif c == 'j':
            	msgJoint.name.append("Elbow")
            	msgJoint.velocity.append(-baseJoint)
            elif c == 'k':
            	msgJoint.name.append("JawRotate")
            	msgJoint.velocity.append(-baseJoint)
            elif c == 'l':
            	msgJoint.name.append("JawOpening")
            	msgJoint.velocity.append(-baseJoint)
            #Increase Velocity send command
            elif c == 'y':
                baseVelocity+=0.1;
                print "baseVelocity: " , baseVelocity
            elif c == 'x':
                if baseVelocity > 0.1:
                    baseVelocity-=0.1
                else:
                    baseVelocity = 0
                print "baseVelocity: " , baseVelocity
            elif c == 'n':
                baseJoint+=0.1;
                print "jointVelocity: " , baseJoint
            elif c == 'm':
                if baseJoint > 0.1:
                    baseJoint -= 0.1
                else:
                    baseJoint = 0
                print "jointVelocity: " , baseJoint
            else:
                print 'wrong key pressed'
            while c!='':
                c = sys.stdin.read(1)
        except IOError: pass

        ##publish the message
        pubTwist.publish(msgTwist)
        pubJoint.publish(msgJoint)
        rospy.sleep(0.1)

##Other input stuff
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
