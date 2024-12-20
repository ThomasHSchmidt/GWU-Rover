#!/usr/bin/env python3

import rospy
from gwurover.msg import RCIN
from gwurover.msg import PID


from std_msgs.msg import Float32




def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def rcin_callback(data): 
    global rcin_msg
    rcin_msg = data


def publish_rcin_data():
    steer_pub = rospy.Publisher("controller/steer", Float32, queue_size=10)
    speed_pub = rospy.Publisher("controller/speed", Float32, queue_size=10)
    
    rate = rospy.Rate(50) # ~50hz
    while not rospy.is_shutdown():    
        steer_pub.publish(map(rcin_msg.ch1,800,2100,-100,100))
        speed_pub.publish(map(rcin_msg.ch2,800,2100,100,-100))
        rate.sleep()




if __name__ == '__main__':
    try:
        rcin_msg = RCIN()
        pid_param_msg = PID()
        rospy.init_node('controller', anonymous=True)
        rospy.Subscriber("rcinput/data", RCIN, rcin_callback)
        
        publish_rcin_data()
    except rospy.ROSInterruptException:
        pass