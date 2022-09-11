import os
import csv

class traj:
    def __init__(self):
        self.base_dir = os.path.dirname(os.path.realpath(__file__))
        self.index_holder = 0
        self.file = None 
        self.csv_file_reader = None
        
    def get_next_waypoint(self): 
        if(self.csv_file_reader):
            return self.csv_file_reader[self.index_holder]
    
    def open_traj_file(self, path):
        self.file = open(os.path.join(os.path.join(self.base_dir,'trajectory'), path + '.csv'), 'r')
        self.csv_file_reader = csv.reader(self.file)
        self.index_holder = 0