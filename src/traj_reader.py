#!/usr/bin/env python
# from __future__ import print_function
from gwurover.srv import WayPoint,WayPointResponse
import rospy

import os
import csv


last_traj_file = None 
first_traj_flag = False
csv_file_reader = None
index_holder = 0

def trajectory_file_reader(req):
    global last_traj_file,first_traj_flag,csv_file_reader,index_holder
    if not first_traj_flag:
        first_traj_flag = True
        index_holder = 0
        file = open(os.path.join(os.path.join(base_dir,'trajectory'), req.traj + '.csv'), 'r')
        _csv_file_reader = csv.reader(file)
        csv_file_reader = []
        for row in _csv_file_reader:
            csv_file_reader.append(row)
        print("here is this",csv_file_reader)
        
    
    if(csv_file_reader):
        #if index_holder == len(csv_file_reader): index_holder = 0
        resp = WayPointResponse(x=csv_file_reader[index_holder][0], y=csv_file_reader[index_holder][1])
        index_holder += 1
        return resp
    else:
        return WayPointResponse(x=0, y=0)




if __name__ == "__main__":
    rospy.init_node('trajectory')
    base_dir = os.path.dirname(os.path.realpath(__file__))
    s = rospy.Service('trajectory/next_waypoint', WayPoint, trajectory_file_reader)
    rospy.spin()
