#!/usr/bin/env python3

import rospy
from gwurover.msg import RCIN
from gwurover.msg import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gwurover.srv import *


from std_msgs.msg import Float32
from tf import euler_from_quaternion

from path_generator import traj
from PID import pid
import math


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def pid_param_callback(data): 
    steer_pid.set_term_p = data.kp
    steer_pid.set_term_i = data.ki
    steer_pid.set_term_d = data.kd

def rcin_callback(data): 
    global rcin_msg
    rcin_msg = data


def t265_position_callback(data):
    global sensor_pos_msg
    sensor_pos_msg = data
    
def t265_velocity_callback(data):
    global sensor_vel_msg
    sensor_vel_msg = data



# def waypoint_callback(data):
#     global waypoint_pos_msg
#     waypoint_pos_msg = data
#     # TODO: reset I term


def algo():
    rate = rospy.Rate(200) # ~200hz
    waypoint_po_x = 1
    waypoint_po_y = 0.5
    while not rospy.is_shutdown():    
        
        if rcin_msg.ch6 > 1000: #the key is on 
            
            
            tetha = math.atan2( waypoint_po_y + sensor_pos_msg.position.x , waypoint_po_x - sensor_pos_msg.position.z) * ( 180 / math.pi )
            dist = math.sqrt( (waypoint_po_y + sensor_pos_msg.position.x) ** 2 + (waypoint_po_x - sensor_pos_msg.position.z) ** 2 )
            if tetha > 90:
                tetha = -(tetha - 180)
                neg = -1
            elif tetha < -90:
                tetha = -(tetha + 180)
                neg = -1
            else: neg = 1
            
            
            if rcin_msg.ch8 > 2000: #speed
                traj_shape = 'circle'
                #print('circle')
            elif rcin_msg.ch8 < 1000:
                traj_shape = 'eight'
                #print('eight')
            else:
                traj_shape = ''
            
            if dist < 0.1:
                #next waypoint
                resp = get_waypoint(traj_shape)
                waypoint_po_x = 2
                waypoint_po_y = 2
                print('new point recived:',waypoint_po_x,waypoint_po_y)
            

            
            
            (roll, pitch, yaw) = euler_from_quaternion ([sensor_pos_msg.orientation.x, sensor_pos_msg.orientation.y, 
                                                            sensor_pos_msg.orientation.z, sensor_pos_msg.orientation.w])
            x_speed = abs(sensor_vel_msg.linear.z * math.cos(pitch)) + abs(sensor_vel_msg.linear.x * math.sin(pitch))
            pitch = -pitch * (180/math.pi)
            
            # steer_pid_value = steer_pid.update_pid(pitch,tetha)
            
            # sspeed = map(rcin_msg.ch3,800,2100,0.1,0.8)
            # speed_pid_value = speed_pid.update_pid(x_speed,sspeed)


            linear_pos_pid_value = linear_pos_pid.update_pid(0,dist)
            if linear_pos_pid_value >= 0:
                    speed_pid_value = speed_pid.update_pid(x_speed,linear_pos_pid_value)
            steer_pid_value = steer_pid.update_pid(pitch,tetha)

            # print("{:.2f}".format(pitch))
            # print(tetha)
            # print(steer_pid_value)

            steer_pub.publish(map(neg * steer_pid_value,-1000,1000,-100,100))
            speed_pub.publish(map(190 + speed_pid_value,-1000,1000,100,-100))
            
            # steer_pub.publish(map(rcin_msg.ch1,800,2100,-100,100))
            # speed_pub.publish(map(rcin_msg.ch2,800,2100,100,-100))
        else:
            steer_pub.publish(map(rcin_msg.ch1,800,2100,-100,100))
            speed_pub.publish(map(rcin_msg.ch2,800,2100,100,-100))
        rate.sleep()





if __name__ == '__main__':
    try:
        rcin_msg = RCIN()
        pid_param_msg = PID()
        sensor_pos_msg = Pose()
        sensor_vel_msg = Twist()
        # waypoint_pos_msg = Pose()
        # waypoint_pos_msg.position.x=1
        # waypoint_pos_msg.position.y=0.5

        rospy.init_node('controller', anonymous=True)
        rospy.Subscriber("controller/pid_params", PID, pid_param_callback)
        rospy.Subscriber("rcinput/data", RCIN, rcin_callback)
        rospy.Subscriber("realsense/pose", Pose, t265_position_callback)
        rospy.Subscriber("realsense/velocity", Twist, t265_velocity_callback)
        # rospy.Subscriber("controller/next_waypoint", Pose, waypoint_callback)
        
        
        steer_pub = rospy.Publisher("controller/steer", Float32, queue_size=10)
        speed_pub = rospy.Publisher("controller/speed", Float32, queue_size=10)
        
        get_waypoint = rospy.ServiceProxy('trajectory/next_waypoint', WayPoint)
        
        steer_pid = pid(70,0,0)
        steer_pid.set_pid_limit(1000)
        steer_pid.set_I_limit(100)
        
        
        
        linear_pos_pid = pid(0.9,0,0.6)
        linear_pos_pid.set_pid_limit(0.4)
        
        speed_pid = pid(250,0,2500)
        speed_pid.set_pid_limit(1000)
        speed_pid.set_I_limit(100)
        
        traj_reader = traj()
        
        algo()
    except rospy.ROSInterruptException:
        pass
