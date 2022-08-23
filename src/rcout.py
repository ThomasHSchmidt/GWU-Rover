#!/usr/bin/env python3

import sys

import rospy
from std_msgs.msg import Float32
import navio.pwm
import navio.util



PWM_speed = 0
PWM_steer = 1



def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def steer_callback(data):        
    pwm_steer.set_duty_cycle(map(data.data,-100,100,1,2))
def speed_callback(data):
    pwm_speed.set_duty_cycle(map(data.data,0,100,1,2))
    
def listener():
    rospy.init_node('rcout', anonymous=True)
    rospy.Subscriber("controller\\steer", Float32, steer_callback)
    rospy.Subscriber("controller\\speed", Float32, speed_callback)
    rospy.spin()

if __name__ == '__main__':
    
    navio.util.check_apm()

    pwm_speed = navio.pwm.PWM(PWM_speed)
    pwm_steer = navio.pwm.PWM(PWM_steer)
    pwm_speed.initialize()
    pwm_speed.set_period(50)
    pwm_speed.enable()
    pwm_steer.initialize()
    pwm_steer.set_period(50)
    pwm_steer.enable()
    
    listener()


