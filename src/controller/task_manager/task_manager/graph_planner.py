from task_manager.worker_manager import Worker
from queue import Queue

class GraphPlanner(object):
    def __init__(self, node):
        self.module_name = 'Graph Planner'
        self.tasks = Queue(0) #infinit size

        self.node = node

    def plan(self, start_node=None, end_node=None):
        self.do_logging('planing')
        self.tasks.put({Worker.NAV:5})
        self.tasks.put({Worker.NAV:3})
        self.tasks.put({Worker.NAV:7})
        self.tasks.put({Worker.NAV:9})
        self.tasks.put({Worker.DOCK:4})
        self.do_logging('planing finished')

    def get(self):
        self.do_logging('peek first one from tasks queue')
        return self.tasks.get_nowait()
        
    def do_logging(self, msg):
        self.node.get_logger().info("[{0}] {1}".format(self.module_name, msg))