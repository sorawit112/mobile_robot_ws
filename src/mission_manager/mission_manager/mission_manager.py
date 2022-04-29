
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from mission_manager.topics import Topics
from mission_manager.mission_executor import Unit, MissionExecutor
from mission_manager.transform_manager import TransformManager
from mission_manager.graph_planner import GraphPlanner
from action_msgs.msg import GoalStatus
from custom_msgs.srv import GetMapMetadata, UserMission
from geometry_msgs.msg import PoseStamped

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.module_name = 'mission_manager'
        self.working = False
        self.map_metadata = None
        self.start_pose = [5.5, 5.5, 0.0] #TODO: get from rosparam instead
        self.last_pose = PoseStamped()

        self.topic = Topics()
        self.transform_manager = TransformManager(self)
        self.graph_planner = GraphPlanner(self, self.transform_manager)
        self.worker_manager = MissionExecutor(self, 
                                        self.transform_manager,
                                        self.worker_actionlib_feedback,
                                        self.worker_actionlib_result) 

        self.getmap_cli = self.create_client(GetMapMetadata, 'get_map_metadata')
        self.request_map_metadata()

        self.usermission_srv = self.create_service(UserMission, self.topic.do_task, self.do_tasks_cb)
              
        self.worker_manager.initial_first_worker()
        self.do_logging('Initialize Completed - ready to receive TASK !!')

    def request_map_metadata(self):
        self.do_logging('wait for get map metdata service')
        srv_ready = self.getmap_cli.wait_for_service()
        if not srv_ready:
            self.do_logging('get map metdata service not ready')
            return 

        self.do_logging('receive response from server')
        future = self.getmap_cli.call_async(GetMapMetadata.Request())
        rclpy.spin_until_future_complete(self, future)

        self.graph_planner.set_map_metadata(future.result().metadata)

    def do_tasks_cb(self, request, response):
        if not self.working:
            self.working = True
            self.do_logging('do tasks')
            
            station_start = request.user_mission.station_start
            station_goal = request.user_mission.station_goal

            current_pose = self.get_current_pose()
            self.graph_planner.plan(current_pose, station_start, station_goal) 

            result = self.worker_manager.request_worker_cancel(Unit.IDLE)
            if result:
                self.do_logging('cancel idle worker completed -> ready to do worker tasks')
            
            response.success = result
            return response
            
    def do_worker_tasks(self):
        self.do_logging('--------------------do worker tasks---------------------------')
        if not self.graph_planner.tasks.empty():
            task = self.graph_planner.get()
            worker = self.worker_manager._dict[task.worker]
            navigate_goal = self.worker_manager.create_navigate_action_goal(task, self.last_pose)
            
            if worker not in list(self.worker_manager.worker_dict.keys()):
                self.do_logging("worker is not registed --> rejected!!!")
                return

            self.do_logging("'{0}' start execute goal from nodes : {1} -> {2}".format(
                                    self.worker_manager.worker_dict[worker].worker_name, 
                                    task.src_node, task.dst_node))
            self.worker_manager.log_worker_action_goal(navigate_goal)

            result = self.worker_manager.request_worker_execution(worker, navigate_goal)

            if not result:
                self.do_logging("'{0}' is not Active set mission abort".format(self.worker_manager.worker_dict[worker].worker_name) )
                
                """ TODO: 
                    1. try to recovery worker from task (restart worker or something)
                    2. excepted return aborted to fleet Management/tasks client
                """
        else:
            self.do_logging("empty task list --> finish job")
            self.worker_manager.active_idle_worker(self.last_pose)
            self.working = False
            self.do_logging("ready to receive new user task")
            
    def worker_actionlib_result(self, future, worker):
        result = future.result().result
        status = future.result().status

        self.last_pose = result.last_pose
        self.do_logging("Action Server Status : {}".format(status))
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.do_logging("{} succeed".format(worker.worker_name))
            self.do_logging("'{}' Received Last Pose: x:{}, y:{}".format(worker.worker_name, 
                                                        result.last_pose.pose.position.x,
                                                        result.last_pose.pose.position.y))
            self.do_logging("")
            self.do_worker_tasks()

        elif status == GoalStatus.STATUS_ABORTED:
            self.do_logging("{} aborted".format(worker.worker_name))
            ## TODO: handeling aborted status

        elif status == GoalStatus.STATUS_CANCELED:
            self.do_logging("{} canceled".format(worker.worker_name))
            if worker.worker_id == Unit.IDLE:
                self.do_worker_tasks()
            ## TODO: handeling canceled status

    def worker_actionlib_feedback(self, feedback, worker):
        self.do_logging("...............'{0}' Feedback: x:{1}, y:{2}".format(worker.worker_name, 
                                                    feedback.feedback.current_pose.pose.position.x,
                                                    feedback.feedback.current_pose.pose.position.y))
    
    def get_current_pose(self):
        current_tf, _ = self.transform_manager.get_tf('map', self.topic.base_footprint)
        current_pose = self.transform_manager.pose_stamped_from_tf_stamped(current_tf)

        return current_pose

    def do_logging(self, msg):
        self.get_logger().info("[{0}] {1}".format(self.module_name, msg))

def main(args=None):
    rclpy.init(args=args)
    try:
        mission_manager = MissionManager()
        
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(mission_manager)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            mission_manager.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()