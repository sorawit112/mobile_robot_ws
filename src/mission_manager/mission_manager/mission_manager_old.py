import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from mission_manager.topics import Topics
from mission_manager.mission_executor import Unit, MissionExecutor
from mission_manager.transform_manager import TransformManager
from mission_manager.graph_planner import GraphPlanner, Task
from action_msgs.msg import GoalStatus
from custom_msgs.srv import GetMapMetadata, UserMission
from geometry_msgs.msg import PoseStamped

NODE_NAME = 'mission_manager'

class MissionManager(Node):
    def __init__(self, start_pose, mission_executor):
        super().__init__(node_name=NODE_NAME)
        self.module_name = 'mission_manager'
        self.working = False
        self.map_metadata = None
        self.do_task_interval = 0.5 #sec
        self.follow_via_point = False
        
        self.start_pose = start_pose #TODO: get from rosparam instead
        self.last_pose = PoseStamped()

        self.topic = Topics()
        self.transform_manager = TransformManager(self)
        self.graph_planner = GraphPlanner(self)

        self.mission_executor = mission_executor

        self.user_mission_srv = self.create_service(UserMission, self.topic.do_task, self.user_mission_cb)

        self.get_map_client = self.create_client(GetMapMetadata, self.topic.get_map_metada)
        self.request_map_metadata()

        self.mission_executor.initial_first_unit()
        self.do_logging('Initialize Completed - ready to receive TASK !!')

    def do_task(self):
        self.do_logging('--------------------do task---------------------------')
        if not self.graph_planner.tasks.empty():
            task = self.graph_planner.get()
            unit = self.unit_from_task(task.unit)
            
            if unit not in list(self.mission_executor.unit_dict.keys()):
                self.do_logging("unit is not registed --> rejected!!!")
                return

            if unit is Unit.NAV and self.follow_via_point:
                task = self.create_followViaPoints_task(task)

            status = self.mission_executor.request_unit_execute(unit, task, self.last_pose)

            if not status:
                self.do_logging("'{0}' not avaliable".format(
                                    self.mission_executor.current_unit.unit_name))
                
                self.graph_planner.clear()
                self.mission_executor.active_idle_unit(self.last_pose)
                """ TODO: 
                    1. try to recovery unit from task (restart unit or something)
                    2. excepted return aborted to fleet Management/tasks client
                """
                
            return True

        else:
            self.do_logging("empty task list --> finish job")
            self.task_timer.destroy()

            self.mission_executor.active_idle_unit(self.last_pose)
            self.working = False
            self.do_logging("ready to receive new user task")

            return False


    ########################################################################################
    #############           Call-Back function
    ########################################################################################
    def user_mission_cb(self, request, response):
        if not self.working:
            self.working = True
            self.do_logging('Receive User Mission -> DO TASKS')
            
            station_start = request.user_mission.station_start
            station_goal = request.user_mission.station_goal

            #cancel idle unit
            result, self.last_pose = self.mission_executor.request_current_unit_cancel() 
            if result:
                self.do_logging('cancel idle unit completed -> ready to do unit tasks')
                
                if not self.map_metadata: #map_metada not None
                    while not self.request_map_metadata(time_out=1):
                        self.do_logging("waitting map_metadata")

                result = self.graph_planner.plan(self.last_pose, station_start, station_goal) 
            
                if result:
                    self.do_task()
                    self.do_logging("Create Timer for checking status of current task")
                    self.task_timer = self.create_timer(self.do_task_interval, self.do_task_interval_cb)


            response.success = result
            return response
        else:
            self.do_logging('Receive request while current user mission is executing -> return Fail')
            response.success = False
            return response

    def do_task_interval_cb(self):
        if not self.mission_executor.current_unit.goal_accepted:
            self.do_logging("goal not accepted")
            return 

        if not self.mission_executor.is_unit_complete(): #tasks not completed
            feedback = self.mission_executor.get_unit_feedback()
            current_pose = feedback.current_pose
            self.do_logging("'{0}' Received Feedback Current Pose: x:{1}, y:{2}".format(
                                        self.mission_executor.current_unit.unit_name, 
                                        current_pose.pose.position.x,
                                        current_pose.pose.position.y))
        else:
            status = self.mission_executor.get_unit_status()
            self.last_pose = self.mission_executor.get_unit_result().last_pose
        
            self.do_logging("Current Task is Completed -> status : {}".format(status))

            self.mission_executor.current_unit.goal_accepted = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.do_logging("'{0}' succeed".format(
                                        self.mission_executor.current_unit.unit_name))
                self.do_logging("'{0}' Received Last Pose: x:{1}, y:{2}".format(
                                        self.mission_executor.current_unit.unit_name, 
                                        self.last_pose.pose.position.x,
                                        self.last_pose.pose.position.y))
                self.do_logging("")

                self.do_task()
                
                return

            elif status == GoalStatus.STATUS_ABORTED:
                self.do_logging("'{0}' aborted".format(
                                        self.mission_executor.current_unit.unit_name))
                ## TODO: handeling aborted status

            elif status == GoalStatus.STATUS_CANCELED:
                self.do_logging("'{0}' canceled".format(
                                        self.mission_executor.current_unit.unit_name))
                ## TODO: handeling canceled status

            if self.mission_executor.current_unit is not Unit.IDLE:
                self.task_timer.destroy()
                
                self.graph_planner.clear()
                self.mission_executor.active_idle_unit(self.last_pose)
                self.do_logging("reset mission manager ready to receive new tasks")
                self.working = False

    ########################################################################################
    #############           Graph Loader, Graph Planner
    ########################################################################################
    def create_followViaPoints_task(self, task):
        self.do_logging("Create follow via-points")
        new_task = Task()
        via_points = []

        peek_task, result = self.graph_planner.peek()
        if not result:
            return task
        peek_unit = self.unit_from_task(peek_task.unit)

        while peek_unit is Unit.NAV:
            next_task = self.graph_planner.get()
            via_points.append(next_task.src_pose)

            peek_task, result = self.graph_planner.peek()
            if not result:
                break
            peek_unit = self.unit_from_task(peek_task.unit)

        if len(via_points != 0):
            last_task = next_task

            new_task.src_node = task.src_node
            new_task.dst_node = last_task.dst_node
            new_task.src_pose = task.src_pose
            new_task.dst_pose = last_task.dst_pose
            new_task.via_points = via_points
            new_task.unit = Unit.NAV
            self.do_logging("Got {} via-points".format(len(via_points)))

            return new_task
        else:
            self.do_logging("No via-points")
            return task

    def request_map_metadata(self, time_out=5):
        self.do_logging('wait for get map metdata service')
        srv_ready = self.get_map_client.wait_for_service(timeout_sec=time_out)
        if not srv_ready:
            self.do_logging('get map metdata service not ready')
            return False

        self.do_logging('receive response from server')
        future = self.get_map_client.call_async(GetMapMetadata.Request())
        rclpy.spin_until_future_complete(self, future)

        self.map_metadata = future.result().metadata

        self.graph_planner.set_map_metadata(self.map_metadata)
        return True


    ########################################################################################
    #############           Utility Function
    ########################################################################################
    @staticmethod
    def unit_from_task(unit):
        unit_dict = {0:Unit.IDLE, 1:Unit.NAV, 2:Unit.DOCK, 3:Unit.TURTLE}

        return unit_dict[unit]

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
    
    def do_logging(self, msg):
        self.get_logger().info("[{0}] {1}".format(self.module_name, msg))


def main(args=None):
    rclpy.init(args=args)
    start_pose = [5.5, 5.5, 0.0]
    try:
        mission_executor = MissionExecutor(start_pose)
        mission_manager = MissionManager(start_pose, mission_executor)
        
        executor = MultiThreadedExecutor(num_threads=5)
        succes1 = executor.add_node(mission_manager)
        succes2 = executor.add_node(mission_executor)
        print(succes1, succes2)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            mission_manager.destroyNode()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()