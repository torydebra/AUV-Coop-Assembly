#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/python

import roslib; roslib.load_manifest('peg')
import rospy
import tf
import sys

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "Insert two frames"
        sys.exit()

    rospy.init_node('tf_a')
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(sys.argv[1], sys.argv[2], rospy.Time(0))     
            print trans, "\n[rpy] ",  tf.transformations.euler_from_quaternion(rot)
            #print "in quaternion:", rot
            print "\n\n"
        except (tf.LookupException, tf.ConnectivityException):
            continue

        rate.sleep()
