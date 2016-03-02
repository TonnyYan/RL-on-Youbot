#! /usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import math
import rospy
import sys
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue

def main(stdscr, persist):
    pub = rospy.Publisher('out',JointPositions, queue_size=1)
    rospy.init_node('bras_teleop', anonymous=True)
    rate = rospy.Rate(20) 
    msg = JointPositions()
    joint = JointValue()
    joint.unit = "rad"
    joint.joint_uri = "arm_joint_1"
    theta1, theta2, theta3, theta4, theta5 = 0.111,0.11,-0.11,0.11,0.111
    jointvalues = [theta1,theta2,theta3,theta4,theta5]
    keycode = -1
    stdscr.addstr("Command\n")
    stdscr.addstr(" - z/s    : theta1 ++/--\n")
    stdscr.addstr(" - e/d    : theta2 ++/--\n ")
    stdscr.addstr(" - r/f    : theta3 ++/--\n")
    stdscr.addstr(" - t/g    : theta4 ++/--\n")
    stdscr.addstr(" - ESC    : exit\n")
    # We will wait for 100 ms for a key to be pressed
    if(persist): stdscr.timeout(100)
    while (not rospy.is_shutdown()) and (keycode != 27): # 27 is escape
        keycode = stdscr.getch() # read pressed key
        if keycode == -1                 : pass # No key has been pressed
        elif keycode == ord('z')   : jointvalues[0]+=0.1
        elif keycode == ord('s')   : jointvalues[0]-=0.1
        elif keycode == ord('e')   : jointvalues[1]+=0.1
        elif keycode == ord('d')   : jointvalues[1]-=0.1
        elif keycode == ord('r')   : jointvalues[2]+=0.1
        elif keycode == ord('f')   : jointvalues[2]-=0.1
        elif keycode == ord('t')   : jointvalues[3]+=0.1
        elif keycode == ord('g')   : jointvalues[3]-=0.1
        for i in range(4): 
            joint.timeStamp = rospy.get_rostime()
            joint.value = jointvalues[i]
            msg.positions.append(joint)
    
        pub.publish(msg)
        rate.sleep()

# Starts curses (terminal handling) and run our main function.
if __name__ == '__main__':
    persist = '--persist' in rospy.myargv(argv=sys.argv)
    try:
        curses.wrapper(lambda w: main(w,persist))
    except rospy.ROSInterruptException:
        pass
