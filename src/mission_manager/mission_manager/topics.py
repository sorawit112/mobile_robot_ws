import os

class Topics(object):
    
    def __init__(self):
        ROBOT_NAME = os.environ['ROBOT_NAME'] #robot1
        self.robot_name = "/" + ROBOT_NAME

        # Frame
        self.odom = ROBOT_NAME + "/odom"
        self.robot_pose = ROBOT_NAME + "/robot_pose"
        self.base_footprint = ROBOT_NAME + "/base_footprint"
        self.base_scan = ROBOT_NAME + "/base_scan"
        
        # Topics
        self.cmd_vel = "/" + ROBOT_NAME + "/cmd_vel"
        
        # Service
        self.do_task = "/" + ROBOT_NAME + "/do_task"
        self.get_map_metada = 'get_map_metadata'
        