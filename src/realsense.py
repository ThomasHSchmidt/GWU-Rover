#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String

import pyrealsense2.pyrealsense2 as rs
import time 


reset_flag = False

def sensor_reset(data):
    global reset_flag
    
    reset_flag = True
    pipe.stop(cfg)
    time.delay(1)
    pipe.start(cfg)
    reset_flag = False

def T265_reader():
    pos_pub = rospy.Publisher('realsense/pose', Pose, queue_size=10)
    vel_pub = rospy.Publisher('realsense/velocity', Twist, queue_size=10)
    vel_msg = Twist()
    pos_msg = Pose()
    rate = rospy.Rate(205) # ~200hz
    while not rospy.is_shutdown():
        if not reset_flag:
            # Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames()
            # Fetch pose frame
            pose = frames.get_pose_frame()
            
            if pose:
                # Print some of the pose data to the terminal
                data = pose.get_pose_data()
                pos_msg.position = data.translation
                pos_msg.orientation = data.rotation
                vel_msg.linear = data.velocity
                pos_pub.publish(pos_msg)
                vel_pub.publish(vel_msg)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        pipe = rs.pipeline()

        # Build config object and request pose data
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        # Start streaming with requested config
        pipe.start(cfg)


        rospy.init_node('realsense', anonymous=True)
        rospy.Subscriber("realsense/reset", String , sensor_reset)
        T265_reader()
    except rospy.ROSInterruptException:
        pipe.stop()
