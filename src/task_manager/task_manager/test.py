import rclpy
from rclpy.time import Duration
from rclpy.time import Time
from builtin_interfaces.msg import Time as time

rclpy.init()
t = rclpy.create_node('test').get_clock().now()
t = time()
t.sec = 1
t.nanosec = 1

T = Time(seconds=t.sec, nanoseconds=t.nanosec)
duration = Duration(seconds=1)

print(type(t), t)
print(type(T), T)
print(type(duration), duration)

print((T+duration).seconds_nanoseconds())