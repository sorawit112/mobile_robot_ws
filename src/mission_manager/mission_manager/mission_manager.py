import rclpy
import time, math

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from mission_manager.topics import Topics
from mission_manager.mission_executor import Unit, MissionExecutor
from mission_manager.transform_manager import TransformManager
from mission_manager.graph_planner import GraphPlanner, Task
from action_msgs.msg import GoalStatus
from custom_msgs.msg import UserMission as dotask
from custom_msgs.srv import GetMapMetadata, UserMission
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int16

NODE_NAME = 'mission_manager'

class MissionManager(Node):
    def __init__(self, mission_executor, start_pose):
        super().__init__(node_name=NODE_NAME)
        self.module_name = 'mission_manager'
        self.working = False
        self.executing = False
        self.current_unit = None
        self.map_metadata = None
        self.do_task_interval = 0.5 #sec
        self.follow_way_point = False

        self.current_pose = start_pose
        self.status = Int16()
        self.status.data = 0
        self.pub_status_once = True

        self.topic = Topics()
        self.transform_manager = TransformManager(self)
        self.graph_planner = GraphPlanner(self)

        self.mission_executor = mission_executor

        self.robot_status_pub = self.create_publisher(Int16, self.topic.robot_status, qos_profile=1)
        
        self.create_subscription(dotask, self.topic.do_task, self.user_mission_cb, qos_profile=1)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose_cb, qos_profile=1)
        self.task_timer = self.create_timer(self.do_task_interval, self.do_task_interval_cb)
        self.info('Initialize Completed - ready to receive TASK !!')

    def do_task_interval_cb(self):
        if self.pub_status_once:
            self.robot_status_pub.publish(self.status)
            self.pub_status_once = False

        if not self.working:
            self.info("No user mission request .....")
            return
        
        if self.graph_planner.tasks.empty():
            self.info("Empty Tasks Queue -> All mission Finish")
            self.working = False
            self.status.data = 0
            self.pub_status_once = True
            return

        if not self.executing:
            self.info('........ do task .......')
            task = self.graph_planner.get()
            unit, unit_name = self.unit_from_task(task.unit)

            self.graph_planner.show_task(task, unit_name)

            goal = self.create_goal_pose_from_task(task)
            if unit is Unit.NAV:
                self.mission_executor.resume_navigator()
                if self.follow_way_point:
                    result, way_points = self.create_followWayPoints_task(task)
                    if result:
                        self.info('do follow_waypoint navigation')
                        status = self.mission_executor.followWaypoints(poses=way_points)
                    else:
                        self.info('do go_to_pose navigation')
                        status = self.mission_executor.goToPose(pose=goal)
                else:
                    self.info('do go_to_pose navigation')
                    status = self.mission_executor.goToPose(pose=goal)
            elif unit is Unit.TURTLE:
                self.info('do go_to_pose Turtle')
                self.mission_executor.pause_navigator()
                status = self.mission_executor.goTurtle(pose=goal)
            elif unit is Unit.DOCK:
                self.info('do go_to_pose Turtle')
                self.mission_executor.pause_navigator()
                status = self.mission_executor.goDocking(pose=goal)
            else:
                status = False
                self.error("wrong unit task")

            if status:
                self.info(f"executing task with unit: {unit_name}")
                self.executing = True
                self.current_unit = unit
            else:
                self.current_unit = None
                self.info("Can not execute task -> clear all task queue")
                self.graph_planner.clear()
            return

        if self.mission_executor.isTaskComplete():
            self.info('Task Completed ..... Getting Status ......')
            status = self.mission_executor.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.info("Task Succeeded")
                self.executing = False
                self.mission_executor.reset_module()
                self.info("reset mission executor .... do next task ....")
                return

            elif status == GoalStatus.STATUS_ABORTED:
                self.error("Task Aborted")
            elif status == GoalStatus.STATUS_CANCELED:
                self.error("Task Canceled")
            else:
                self.error("Task Unknown")

            self.info('reset mission executor and clear task queue')
            self.executing = False
            self.graph_planner.clear()
            self.mission_executor.reset_module()
            self.status.data = 2
            self.pub_status_once = True
            return

        feedback = self.mission_executor.getFeedback()
        if feedback:
            if self.current_unit is Unit.NAV:
                if self.follow_way_point:
                    current_waypoint = feedback.current_waypoint
                    self.info(f'[feedback] current_waypoint: {current_waypoint}')
                else:
                    self.current_pose = feedback.current_pose
                    pose = self.current_pose.pose
                    self.info('[feedback] current_pose: x:{:.2f}, y:{:.2f}'.format(
                                                    pose.position.x,
                                                    pose.position.y
                                                    ))
            elif self.current_unit is Unit.DOCK:
                self.current_pose = feedback.current_pose
                pose = self.current_pose.pose
                self.info('[feedback] current_pose: x:{:.2f}, y:{:.2f}'.format(
                                                    pose.position.x,
                                                    pose.position.y
                                                    ))
            elif self.current_unit is Unit.TURTLE:
                self.current_pose = feedback.current_pose
                pose = self.current_pose.pose
                self.info('[feedback] current_pose: x:{:.2f}, y:{:.2f}'.format(
                                                    pose.position.x,
                                                    pose.position.y
                                                    ))
        else:
            self.warn("no feedback from action server")
        
    ########################################################################################
    #############           Call-Back function
    ########################################################################################
    def user_mission_cb(self, msg):
        if not self.working:
            self.info('Receive User Mission -> DO TASKS')
            
            node_list = msg.node_list
            self.map_metadata = msg.map_metadata

            self.graph_planner.set_map_metadata(self.map_metadata)

            result = self.graph_planner.plan(self.current_pose, node_list) 
        
            if result:
                self.working = True
                self.status.data = 1
                self.pub_status_once = True
            else:
                self.working = False
                self.status.data = 2
                self.pub_status_once = True
                self.error("Plan Failed!!!")
            
            self.info(f"send response to client : {result}")
            # response.success = result
            # return response
        else:
            self.warn('Receive request while current user mission is executing -> return Fail')
            # response.success = False
            # return response

    def initial_pose_cb(self, msg):
        self.info('receive initial_pose ... setting current pose')
        self.info('current_pose -> x :{:.2f}, y: {:.2f}'.format(
                                        msg.pose.pose.position.x, 
                                        msg.pose.pose.position.y))
        self.current_pose.pose = msg.pose.pose

    ########################################################################################
    #############           Graph Loader, Graph Planner
    ########################################################################################
    def create_followWayPoints_task(self, task):
        self.info("Create follow waypoints")
        way_points = []

        peek_task, result = self.graph_planner.peek()
        if not result:
            return task
        peek_unit = self.unit_from_task(peek_task.unit)

        while peek_unit is Unit.NAV:
            next_task = self.graph_planner.get()
            way_points.append(next_task.src_pose)

            peek_task, result = self.graph_planner.peek()
            if not result:
                break
            peek_unit = self.unit_from_task(peek_task.unit)

        if len(way_points != 0):
            last_task = next_task

            way_points.append(last_task.dst_pose)
            self.info("Got {} waypoints".format(len(way_points)))

            return way_points
        else:
            self.info("No way-points")
            return False, None

    ########################################################################################
    #############           Utility Function
    ########################################################################################
    def create_goal_pose_from_task(self, task):
        dst_pose = PoseStamped()
        dst_pose.header.stamp = self.get_clock().now().to_msg()
        dst_pose.header.frame_id = "map"
        dst_pose.pose.position.x = task.dst_pose[0]
        dst_pose.pose.position.y = task.dst_pose[1]
        dst_pose.pose.position.z = 0.0

        self.info('src_pose.x:{}, src_pose.y{}'.format(task.src_pose[0], task.src_pose[1]))
        self.info('dst_pose.x:{}, dst_pose.y{}'.format(task.dst_pose[0], task.dst_pose[1]))

        x,y,z,w = self.calculate_heading(task.src_pose, task.dst_pose)

        dst_pose.pose.orientation.x = x
        dst_pose.pose.orientation.y = y
        dst_pose.pose.orientation.z = z
        dst_pose.pose.orientation.w = w

        return dst_pose

    @staticmethod
    def calculate_heading(src_pose, dst_pose):
        y_diff = dst_pose[1] - src_pose[1]
        x_diff = dst_pose[0] - src_pose[0]

        theta = math.atan2(y_diff, x_diff)

        print('dst_angle:{}'.format(theta*180/math.pi))

        x,y,z,w = TransformManager.quaternion_from_euler(0,0,theta)

        return x,y,z,w

    @staticmethod
    def unit_from_task(unit):
        unit_dict = {1:Unit.NAV, 2:Unit.DOCK, 3:Unit.TURTLE}
        unit_name = {1:'Nav', 2:'Dock', 3:'Turtle'}

        return unit_dict[unit], unit_name[unit]

    def get_current_pose(self):
        current_tf, _ = self.transform_manager.get_tf('map', self.topic.base_footprint)
        current_pose = self.transform_manager.pose_stamped_from_tf_stamped(current_tf)

        return current_pose

    def destroyNode(self):
        self.mission_executor.destroyNode()
        super().destroy_node()


    ########################################################################################
    #############           Logging
    ########################################################################################
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(args=None):
    rclpy.init(args=args)
    # start_pose = [10.5, 9.0, math.pi/2]
    start_pose = [-0.5, 0.0, 0.0]

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.position.x = start_pose[0]
    initial_pose.pose.position.y = start_pose[1]
    initial_pose.pose.position.z = 0.0
    x,y,z,w = TransformManager.quaternion_from_euler(0, 0, start_pose[2])
    initial_pose.pose.orientation.x = x
    initial_pose.pose.orientation.y = y
    initial_pose.pose.orientation.z = z
    initial_pose.pose.orientation.w = w

    try:
        mission_executor = MissionExecutor()

        # mission_executor.setInitialPose(initial_pose)
        # mission_executor.waitUntilNav2Active()

        # time.sleep(1)

        mission_manager = MissionManager(mission_executor, initial_pose)

        executor = MultiThreadedExecutor(num_threads=5)
        succes1 = executor.add_node(mission_manager)
        succes2 = executor.add_node(mission_executor)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            mission_manager.destroyNode()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()