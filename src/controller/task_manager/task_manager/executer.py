from random import randint
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from task_manager.worker_manager import Worker, WorkerManager
from task_manager.graph_planner import GraphPlanner
from std_srvs.srv import Empty
from std_msgs.msg import Empty

MAP_NAME = "map_01" #TODO: use rosparam instead

class Executer(Node):
    def __init__(self):
        super().__init__('executer')
        self.module_name = 'Executer'
        self.active = False
            
        self.graph_planner = GraphPlanner(self, MAP_NAME)
        self.worker_manager = WorkerManager(self, 
                                        self.worker_actionlib_feedback,
                                        self.worker_actionlib_result) 

        self.create_subscription(Empty, '/do_tasks', self.do_tasks, 1)
        
        self.initial_node = randint(min(self.graph_planner.graph.nodes), max(self.graph_planner.graph.nodes))
        self.worker_manager.request_worker_execution(Worker.IDLE, goal=10)

    def do_tasks(self, _):
        if not self.active:
            self.do_logging('do tasks')
            self.active = True
            self.graph_planner.plan()
            result = self.worker_manager.request_worker_cancel(Worker.IDLE)
            if result:
                self.do_logging('canecel idle worker completed -> ready to do worker tasks')

    def do_worker_tasks(self):
        self.do_logging('do worker tasks')
        if not self.graph_planner.tasks.empty():
            task = self.graph_planner.get()
            # worker,goal = next(iter(task.items()))
            node_start = task.node_start
            node_goal = task.node_goal
            worker = task.worker

            self.do_logging("'{0}' start execute goal : {1}".format(self.worker_manager.worker_dict[worker].worker_name, node_goal))

            if worker not in [Worker.IDLE, Worker.NAV, Worker.DOCK]:
                self.do_logging("worker is not registed --> rejected!!!")
                return

            result = self.worker_manager.request_worker_execution(worker, node_goal)

            if not result:
                self.do_logging("'{0}' is not Active set mission abort".format(self.worker_manager.worker_dict[worker].worker_name) )
            
        else:
            self.do_logging("empty task list --> finish job")
            self.active = False
            self.worker_manager.request_worker_execution(Worker.IDLE)

    def worker_actionlib_result(self, future, worker):
        result = future.result().result
        self.do_logging("'{}' Received Result: {}".format(worker.worker_name, result.sequence))
        worker._executing = False

        self.do_worker_tasks()

    def worker_actionlib_feedback(self, feedback, worker):
        self.do_logging("'{}' Received Feedback: {}".format(worker.worker_name, feedback.feedback.partial_sequence))

    def do_logging(self, msg):
        self.get_logger().info("[{0}] {1}".format(self.module_name, msg))

def main(args=None):
    rclpy.init(args=args)

    action_client = Executer()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()