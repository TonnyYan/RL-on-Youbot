#! /usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import math
import copy
import rospy
import sys
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue

def main(stdscr, persist):
    pub = rospy.Publisher('out',JointPositions, queue_size=1)
    rospy.init_node('bras_teleop', anonymous=True)
    theta1, theta2, theta3, theta4, theta5 = 0.111,0.11,-0.11,0.11,0.111
    jointvalues = [theta1,theta2,theta3,theta4,theta5]
    keycode = -1
    rate = rospy.Rate(100) 
    stdscr.addstr("Command\n")
    stdscr.addstr("- a/q    : theta1 ++/--\n")
    stdscr.addstr("- z/s    : theta2 ++/--\n")
    stdscr.addstr("- e/d    : theta3 ++/--\n")
    stdscr.addstr("- r/f    : theta4 ++/--\n")
    stdscr.addstr("- t/g    : theta5 ++/--\n")
    stdscr.addstr("- ESC    : exit\n")
    # We will wait for 100 ms for a key to be pressed
    if(persist): stdscr.timeout(100)
    while (not rospy.is_shutdown()) and (keycode != 27): # 27 is escape
        keycode = stdscr.getch() # read pressed key
        if keycode == -1  : pass # No key has been pressed
        elif keycode == ord('a')   : 
            if(jointvalues[0]+0.05> 0.0100692 and jointvalues[0]+0.05<5.84014 ):jointvalues[0]+=0.05
        elif keycode == ord('q')   : 
            if(jointvalues[0]-0.05>0.0100692 and jointvalues[0]-0.05< 5.84014 ):jointvalues[0]-=0.05  
        elif keycode == ord('z')   : 
            if(jointvalues[1]+0.05> 0.0100692 and jointvalues[1]+0.05< 2.61799 ):jointvalues[1]+=0.05
        elif keycode == ord('s')   :
            if(jointvalues[1]-0.05>0.0100692 and jointvalues[1]-0.05<  2.61799  ):jointvalues[1]-=0.05
        elif keycode == ord('e')   :
            if(jointvalues[2]+0.05>-5.02655 and jointvalues[2]+0.05<-0.015708  ):jointvalues[2]+=0.05
        elif keycode == ord('d')   :
            if(jointvalues[2]-0.05>-5.02655 and jointvalues[2]-0.05<-0.015708  ):jointvalues[2]-=0.05
        elif keycode == ord('r')   :
            if(jointvalues[3]+0.05>0.0221239 and jointvalues[3]+0.05<3.4292 ):jointvalues[3]+=0.05
        elif keycode == ord('f')   :
            if(jointvalues[3]-0.05>0.0221239 and jointvalues[3]-0.05<3.4292 ):jointvalues[3]-=0.05
        elif keycode == ord('t')   : jointvalues[4]+=0.05
        elif keycode == ord('g')   : jointvalues[4]-=0.05
        jp = JointPositions()
        jv1 = JointValue()
        jv2 = JointValue()
        jv3 = JointValue()
        jv4 = JointValue()
        jv5 = JointValue()
        jv1.joint_uri ="arm_joint_1"
        jv1.unit = "rad"
        jv1.value =jointvalues[0]
        jv2.joint_uri ="arm_joint_2"
        jv2.unit = "rad"
        jv2.value = jointvalues[1]
        jv3.joint_uri = "arm_joint_3"
        jv3.unit = "rad"
        jv3.value = jointvalues[2]
        jv4.joint_uri = "arm_joint_4"
        jv4.unit = "rad"
        jv4.value = jointvalues[3]
        jv5.joint_uri = "arm_joint_5"
        jv5.unit = "rad"
        jv5.value = jointvalues[4]
        jp.positions.append(copy.deepcopy(jv1))
        jp.positions.append(copy.deepcopy(jv2))
        jp.positions.append(copy.deepcopy(jv3))
        jp.positions.append(copy.deepcopy(jv4))
        jp.positions.append(copy.deepcopy(jv5))
        pub.publish(jp)
        rate.sleep()

# Starts curses (terminal handling) and run our main function.
if __name__ == '__main__':
    persist = '--persist' in rospy.myargv(argv=sys.argv)
    try:
        curses.wrapper(lambda w: main(w,persist))
    except rospy.ROSInterruptException:
        pass
