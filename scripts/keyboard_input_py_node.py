#!/usr/bin/env python2

import sys, select, termios, tty

import rospy
from std_msgs.msg import Int32
#from __future__ import print_function

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

keyboard_pub_ = rospy.Publisher('/keyboard_input_py/raw_input', Int32, queue_size=1)

if __name__ == "__main__":

    rospy.init_node('KeyboardInputPyNode')

    settings = termios.tcgetattr(sys.stdin)

    #KeyboardInputPyNode()
    r = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        key = getKey()
        sys.stdout.write(key)
        keyboard_pub_.publish(Int32(data = ord(key)))
        if ord(key) == 96: #This is escape, the tilde squiqly key
            break
        r.sleep()

    # rospy.spin() #you can get rid of this spin if you want to totally kill it
