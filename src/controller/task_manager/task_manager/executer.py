import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from task_manager.worker_manager import Worker, WorkerManager
from task_manager.transform_manager import TransformManager
from task_manager.graph_planner import GraphPlanner
from std_srvs.srv import Empty
from std_msgs.msg import Empty
from custom_msgs.srv import GetMapMetadata
from geometry_msgs.msg import PoseStamped

class Executer(Node):
    def __init__(self):
        super().__init__('executer')
        self.module_name = 'Executer'
        self.active = False
        self.map_metadata = None
        self.start_pose = [0,0] #TODO: get from rosparam instead
        self.last_pose = PoseStamped()

        self.transform_manager = TransformManager(self)
        self.graph_planner = GraphPlanner(self, self.transform_manager)
        self.worker_manager = WorkerManager(self, 
                                        self.transform_manager,
                                        self.worker_actionlib_feedback,
                                        self.worker_actionlib_result) 

        self.getmap_cli = self.create_client(GetMapMetadata, 'get_map_metadata')
        self.request_map_metadata()

        self.create_subscription(Empty, '/do_tasks', self.do_tasks, 1)
              
        self.worker_manager.initial_first_worker()
        
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

    def do_tasks(self, _):
        if not self.active:
            self.active = True
            self.do_logging('do tasks')

            result = self.worker_manager.request_worker_cancel(Worker.IDLE)
            if result:
                self.do_logging('canecel idle worker completed -> ready to do worker tasks')
            
            current_pose = self.get_current_pose()
            self.graph_planner.plan(current_pose, 'A', 'C')
            
    def do_worker_tasks(self):
        self.do_logging('do worker tasks')
        if not self.graph_planner.tasks.empty():
            task = self.graph_planner.get()
            worker = task.worker
            navigate_goal = self.worker_manager[worker].create_navigate_action_goal(task, self.last_pose)

            if worker not in list(Worker._dict.values()):
                self.do_logging("worker is not registed --> rejected!!!")
                return

            self.do_logging("'{0}' start execute goal from nodes : {1} -> {2}".format(
                                    self.worker_manager.worker_dict[worker].worker_name, 
                                    task.src_node, task.dst_node))
            self.worker_manager[worker].log_worker_action_goal(navigate_goal)

            result = self.worker_manager.request_worker_execution(worker, navigate_goal)

            if not result:
                self.do_logging("'{0}' is not Active set mission abort".format(self.worker_manager.worker_dict[worker].worker_name) )
                
                """ TODO: 
                    1. try to recovery worker from task (restart worker or something)
                    2. excepted return aborted to fleet Management/tasks client
                """
        else:
            self.do_logging("empty task list --> finish job")
            self.active = False
            self.worker_manager.active_idle_worker(self.last_pose)

    def worker_actionlib_result(self, future, worker):
        result = future.result().result
        self.last_pose = result.last_pose
        self.do_logging("'{}' Received Last Pose: {}".format(worker.worker_name, result.last_pose))
        
        self.do_worker_tasks()

    def worker_actionlib_feedback(self, feedback, worker):
        self.do_logging("'{}' Received Feedback: {}".format(worker.worker_name, feedback.feedback.partial_sequence))
    
    def get_current_pose(self):
        current_tf = self.transform_manager.get_tf('map', 'base_footprint')
        current_pose = self.transform_manager.pose_stamped_from_tf_stamped(current_tf)

        return current_pose

    def do_logging(self, msg):
        self.get_logger().info("[{0}] {1}".format(self.module_name, msg))

def main(args=None):
    rclpy.init(args=args)

    action_client = Executer()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()