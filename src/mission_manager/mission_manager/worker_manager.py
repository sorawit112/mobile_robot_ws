import math
from task_manager.topics import Topics
from rclpy.action import ActionClient
from custom_msgs.action import NavigateAction
from task_manager.transform_manager import TransformManager
from geometry_msgs.msg import PoseStamped
from enum import Enum

class Worker(Enum):
    """ workers instance list"""
    IDLE = 0
    NAV = 1
    DOCK = 2
    TURTLE = 3

    _dict = {'IDLE':IDLE, 'NAV':NAV, 'DOCK':DOCK, 'TURTLE':TURTLE}

class WorkerEntry(object):
    def __init__(self,
                    node, 
                    worker_name, 
                    worker_id, 
                    action_spec, 
                    result_callback,
                    feedback_callback,
                    ):
        self.module_name = "Worker Entry"
        self.node = node
        self._executing = False
        self.worker_name = worker_name
        self.action_spec = action_spec
        self.worker_id = worker_id
        self.result_callback = result_callback
        self.feedback_callback = feedback_callback
        self._goal_handle = None
        self.action_topic = node.topic.robot_name + "/" + str(self.worker_name) + "_server"
        self.action_client = ActionClient(node, self.action_spec, self.action_topic)

    def is_server_ready(self):
        result = self.action_client.wait_for_server(5)
        if not result:
            self.do_logging('Server ' + str(self.action_topic) + ' Not Active')
            return False
        return True

    def cancel_goal(self):
        self.do_logging('canceling current goal')
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)
            return True
        else:
            self.do_logging('request cancel while no goal is executing')
            return False

    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.do_logging('Goal successfully canceled')
            self._executing = False
        else:
            self.do_logging('Goal failed to cancel')

    def send_goal(self, goal_msg):
        self.do_logging('send goal to ' + str(self.action_topic))

        result = self.is_server_ready()
        if not result:
            return False

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.wrapped_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.do_logging('Goal rejected :(')
            return

        self._goal_handle = goal_handle
        self._executing = True
        
        self.do_logging('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.wrapped_result_callback)

    def wrapped_result_callback(self, future):
        self._goal_handle = None
        self.result_callback(future, self)

    def wrapped_feedback_callback(self, feedback):
        self.feedback_callback(feedback, self)

    def do_logging(self, msg):
        self.node.get_logger().info("[{0}/{1}] {2}".format(self.module_name, self.worker_name, msg))


class WorkerManager(object):
    def __init__(self, node, transformanager, feedback_cb, result_cb):
        self.module_name = 'Worker_Manager'
        self.worker_dict = {}
        self._dict = {0:Worker.IDLE, 1:Worker.NAV, 2:Worker.DOCK, 3:Worker.TURTLE}
        self.worker_executing = None
        self.transform_manager = transformanager

        self.node = node
        self.feedback_cb = feedback_cb
        self.result_cb = result_cb

        self.create_workers()

    def request_worker_execution(self, worker, goal):
        self.worker_executing = self.worker_dict[worker]
        
        self.do_logging("request '{}' to exection".format(self.worker_executing.worker_name))

        return self.worker_executing.send_goal(goal)

    def request_worker_cancel(self, worker):
        worker = self.worker_dict[worker]

        self.do_logging("request '{}' to cancel".format(worker.worker_name))

        return worker.cancel_goal()
    
    def active_idle_worker(self, last_pose):
        idle_worker = self.worker_dict[Worker.IDLE]

        self.do_logging("active idle worker")
        navigate_goal = NavigateAction.Goal()
        navigate_goal.src_pose = last_pose
        navigate_goal.src_pose.header.stamp = self.node.get_clock().now().to_msg()
        
        idle_worker.send_goal(navigate_goal)

    def initial_first_worker(self):
        initial_worker = self.worker_dict[Worker.IDLE]
        server_ready = initial_worker.is_server_ready()

        if not server_ready:
            self.do_logging("initial first worker FAILED!!!")
            return
        
        self.do_logging("initial first worker")
        navigate_goal = NavigateAction.Goal()
        navigate_goal.src_pose.header.stamp = self.node.get_clock().now().to_msg()
        navigate_goal.src_pose.header.frame_id = "map"
        navigate_goal.src_pose.pose.position.x = self.node.start_pose[0]
        navigate_goal.src_pose.pose.position.y = self.node.start_pose[1]

        x,y,z,w = self.transform_manager.quaternion_from_euler(0., 0., self.node.start_pose[2])
        navigate_goal.src_pose.pose.orientation.x = x
        navigate_goal.src_pose.pose.orientation.y = y
        navigate_goal.src_pose.pose.orientation.z = z
        navigate_goal.src_pose.pose.orientation.w = w

        initial_worker.send_goal(navigate_goal)

    def create_workers(self):
        idle_worker = WorkerEntry(
                            self.node,
                            "idle_worker",
                            worker_id=Worker.IDLE,
                            action_spec=NavigateAction,
                            result_callback=self.result_cb,
                            feedback_callback=self.feedback_cb
                            )

        nav_worker = WorkerEntry(
                            self.node,
                            "nav_worker",
                            worker_id=Worker.NAV,
                            action_spec=NavigateAction,
                            result_callback=self.result_cb,
                            feedback_callback=self.feedback_cb
                            )

        docking_worker = WorkerEntry(
                            self.node,
                            "docking_worker",
                            worker_id=Worker.DOCK,
                            action_spec=NavigateAction,
                            result_callback=self.result_cb,
                            feedback_callback=self.feedback_cb
                            )

        turtle_worker = WorkerEntry(self.node,
                            "turtle_worker",
                            worker_id=Worker.TURTLE,
                            action_spec=NavigateAction,
                            result_callback=self.result_cb,
                            feedback_callback=self.feedback_cb
                            )

        self.worker_dict[Worker.IDLE] = idle_worker
        self.worker_dict[Worker.NAV] = nav_worker
        self.worker_dict[Worker.DOCK] = docking_worker
        self.worker_dict[Worker.TURTLE] = turtle_worker

    def create_navigate_action_goal(self, task, src_pose):
        goal = NavigateAction.Goal()
        goal.src_node = task.src_node
        goal.dst_node = task.dst_node

        src_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.src_pose = src_pose
        
        dst_pose = PoseStamped()
        dst_pose.header.stamp = self.node.get_clock().now().to_msg()
        dst_pose.header.frame_id = "map"
        dst_pose.pose.position.x = task.dst_pose[0]
        dst_pose.pose.position.y = task.dst_pose[1]
        dst_pose.pose.position.z = 0.0

        goal.dst_pose = self.calculate_heading(src_pose, dst_pose)

        return goal

    def calculate_heading(self, src_pose, dst_pose):
        y_diff = dst_pose.pose.position.y - dst_pose.pose.position.y
        x_diff = src_pose.pose.position.x - src_pose.pose.position.x

        theta = math.atan2(y_diff, x_diff)

        x,y,z,w = self.transform_manager.quaternion_from_euler(0,0,theta)

        dst_pose.pose.orientation.x = x
        dst_pose.pose.orientation.y = y
        dst_pose.pose.orientation.z = z
        dst_pose.pose.orientation.w = w

        return dst_pose

    def log_worker_action_goal(self, goal):
        self.do_logging("FROM : [{}] ({}, {})".format(
            goal.src_node,
            goal.src_pose.pose.position.x,
            goal.src_pose.pose.position.y,
        ))

        self.do_logging("TO : [{}] ({}, {})".format(
            goal.dst_node,
            goal.dst_pose.pose.position.x,
            goal.dst_pose.pose.position.y,
        ))

    def do_logging(self, msg):
        self.node.get_logger().info("[{0}] {1}".format(self.module_name, msg))