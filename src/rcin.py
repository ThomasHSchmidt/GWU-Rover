#!/usr/bin/env python3

import sys

import rospy
from gwurover.msg import RCIN

import navio.util
import navio.rcinput


def radio_publisher():
    rcin_pub = rospy.Publisher('rcinput/data', RCIN, queue_size=10)
    rate = rospy.Rate(105) # ~100hz
    while not rospy.is_shutdown():
        rcin_msg.ch1 = int(rcin.read(0))
        rcin_msg.ch2 = int(rcin.read(1))
        rcin_msg.ch3 = int(rcin.read(2))
        rcin_msg.ch4 = int(rcin.read(3))
        rcin_msg.ch5 = int(rcin.read(4))
        rcin_msg.ch6 = int(rcin.read(5))
        rcin_msg.ch7 = int(rcin.read(6))
        rcin_msg.ch8 = int(rcin.read(7))
        rcin_pub.publish(rcin_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rcin_msg = RCIN()
        rcin = navio.rcinput.RCInput()
        rospy.init_node('rcinput', anonymous=True)
        radio_publisher()
    except rospy.ROSInterruptException:
        pass