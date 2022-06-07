import os

class Topics(object):
    
    def __init__(self):
        ROBOT_NAME = os.environ['ROBOT_NAME'] #robot1
        self.robot_name = "/" + ROBOT_NAME

        # Frame
        # self.odom = ROBOT_NAME + "/odom"
        # self.robot_pose = ROBOT_NAME + "/robot_pose"
        # self.base_footprint = ROBOT_NAME + "/base_footprint"
        # self.base_scan = ROBOT_NAME + "/base_scan"
        self.odom = "odom"
        self.base_footprint = "base_footprint"
        
        # Topics
        self.cmd_vel = "/cmd_vel"

        self.amcl_pose = "/amcl_pose"
        
        self.status = "/" + ROBOT_NAME + "/status" 
        self.robot_status = "/" + ROBOT_NAME + "/robot_status" 
        self.current_pose = "/" + ROBOT_NAME + "/current_pose" 
        self.do_task = "/" + ROBOT_NAME + "/do_task"
        
        # Service
        self.do_usermission = "/" + ROBOT_NAME + "/do_usermission"
        self.get_map_metada = 'get_map_metadata'
        