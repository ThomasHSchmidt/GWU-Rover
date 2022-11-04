#!/usr/bin/env python3

import rospy
from gwurover.msg import RCIN
from gwurover.msg import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gwurover.srv import *


from std_msgs.msg import Float32, String
from tf import euler_from_quaternion

from path_generator import traj
from PID import pid
import math


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def pid_param_callback(data):
    if data.kp_steer >= 0: steer_pid.set_term_kp(data.kp_steer)
    if data.ki_steer >= 0: steer_pid.set_term_ki(data.ki_steer)
    if data.kd_steer >= 0: steer_pid.set_term_kd(data.kd_steer)
    if data.offset_steer >= 0: steer_pid.set_term_offset(data.offset_steer)
    
    if data.kp_speed >= 0: speed_pid.set_term_kp(data.kp_speed)
    if data.ki_speed >= 0: speed_pid.set_term_ki(data.ki_speed)
    if data.kd_speed >= 0: speed_pid.set_term_kd(data.kd_speed)
    if data.offset_speed >= 0: speed_pid.set_term_offset(data.offset_speed)

def rcin_callback(data): 
    global rcin_msg
    rcin_msg = data


def t265_position_callback(data):
    global sensor_pos_msg
    sensor_pos_msg = data
    ########################################################### comment this for the test rover 
    sensor_pos_msg.position.x *= -1
    sensor_pos_msg.position.z *= -1
    ###########################################################

def t265_velocity_callback(data):
    global sensor_vel_msg
    sensor_vel_msg = data





def algo():
    rate = rospy.Rate(200) # ~200hz
    waypoint_po_x = 0
    waypoint_po_y = 0
    sensor_reset_key_on_flag = False
    traj_shape = ""
    while not rospy.is_shutdown():    
        
        if rcin_msg.ch6 > 1000: #the key is on 
            
            
            tetha = math.atan2( waypoint_po_y + sensor_pos_msg.position.x , waypoint_po_x - sensor_pos_msg.position.z) * ( 180 / math.pi )
            dist = math.sqrt( (waypoint_po_y + sensor_pos_msg.position.x) ** 2 + (waypoint_po_x - sensor_pos_msg.position.z) ** 2 )

            
            
            if rcin_msg.ch8 > 2000: #speed
                if traj_shape != 'circle':
                    traj_shape = 'circle'
                    dist = 0
            elif rcin_msg.ch8 < 1000:
                if traj_shape != 'eight':
                    traj_shape = 'eight'
                    dist = 0
            else:
                if traj_shape != 'MP':
                    traj_shape = 'MP'
                    dist = 0
           
            if dist < 0.2:
                #next waypoint
                resp = get_waypoint(traj_shape)
                waypoint_po_x = resp.x
                waypoint_po_y = resp.y
                print('new point recived:',waypoint_po_x,waypoint_po_y)
            

            
            
            (roll, pitch, yaw) = euler_from_quaternion ([sensor_pos_msg.orientation.x, sensor_pos_msg.orientation.y, 
                                                            sensor_pos_msg.orientation.z, sensor_pos_msg.orientation.w])
            x_speed = abs(sensor_vel_msg.linear.z * math.cos(pitch)) + abs(sensor_vel_msg.linear.x * math.sin(pitch))
            pitch = -pitch * (180/math.pi)
            yaw = yaw * (180/math.pi)
            if abs(yaw) > 100:pitch = (180 - abs(pitch)) * abs(pitch)/pitch
            print("{:3.1f}".format(tetha),
                          "{:3.1f}".format(pitch),
                          "{:3.1f}".format(sensor_pos_msg.position.z),
                          "{:3.1f}".format(-sensor_pos_msg.position.x), end='\r')
            

            if abs(pitch) > 90 and abs(tetha) > 90 and pitch*tetha < 0: neg = -1
            else: neg = 1

            linear_pos_pid_value = linear_pos_pid.update_pid(0,dist)
            if linear_pos_pid_value >= 0:
                    speed_pid_value = speed_pid.update_pid(x_speed,0.4)
            steer_pid_value = steer_pid.update_pid(pitch,tetha)

            
            steer_pub.publish(map(neg * steer_pid_value,-1000,1000,-100,100))
            speed_pub.publish(map(speed_pid_value,-1000,1000,100,-100))
            
            # steer_pub.publish(map(rcin_msg.ch1,800,2100,-100,100))
            # speed_pub.publish(map(rcin_msg.ch2,800,2100,100,-100))
        else:
            # if rcin_msg.ch5 > 1000 and not sensor_reset_key_on_flag:
            #     sensor_reset_key_on_flag = True
            #     sensor_reset_pub.publish('')
            # else: 
            #     sensor_reset_key_on_flag = False
            steer_pub.publish(map(rcin_msg.ch1,800,2100,-100,100))
            speed_pub.publish(map(rcin_msg.ch2,800,2100,100,-100))
        rate.sleep()





if __name__ == '__main__':
    try:
        rcin_msg = RCIN()
        pid_param_msg = PID()
        sensor_pos_msg = Pose()
        sensor_vel_msg = Twist()

        rospy.init_node('controller', anonymous=True)
        rospy.Subscriber("controller/pid_params", PID, pid_param_callback)
        rospy.Subscriber("rcinput/data", RCIN, rcin_callback)
        rospy.Subscriber("realsense/pose", Pose, t265_position_callback)
        rospy.Subscriber("realsense/velocity", Twist, t265_velocity_callback)
        
        
        steer_pub = rospy.Publisher("controller/steer", Float32, queue_size=10)
        speed_pub = rospy.Publisher("controller/speed", Float32, queue_size=10)
        sensor_reset_pub = rospy.Publisher("realsense/reset", String, queue_size=10)
        
        rospy.wait_for_service('trajectory/next_waypoint')
        get_waypoint = rospy.ServiceProxy('trajectory/next_waypoint', WayPoint)
        
        steer_pid = pid(70,0,0)
        steer_pid.set_pid_limit(1000)
        steer_pid.set_I_limit(100)
        
        
        
        linear_pos_pid = pid(1,0,0.35)
        linear_pos_pid.set_pid_limit(0.4)
        
        speed_pid = pid(250,0,2500,offset=190)
        speed_pid.set_pid_limit(1000)
        speed_pid.set_I_limit(100)
        
        traj_reader = traj()
        
        algo()
    except rospy.ROSInterruptException:
        pass
