export server_ip=172.20.10.3
export server_port=10000
export robot_port1=20000
export robot_port2=30000
export robot_port3=40000

alias dds_robot1='export ROS_DISCOVERY_SERVER=";$server_ip:$robot_port1;;"'
alias dds_robot2='export ROS_DISCOVERY_SERVER=";;$server_ip:$robot_port2;"'
alias dds_robot3='export ROS_DISCOVERY_SERVER=";;;$server_ip:$robot_port3"'
alias dds_adapter1='export ROS_DISCOVERY_SERVER="$server_ip:$server_port;$server_ip:$robot_port1;;"'
alias dds_adapter2='export ROS_DISCOVERY_SERVER="$server_ip:$server_port;;$server_ip:$robot_port2;"'
alias dds_adapter3='export ROS_DISCOVERY_SERVER="$server_ip:$server_port;;;$server_ip:$robot_port3"'
alias dds='echo $ROS_DISCOVERY_SERVER'

alias dds_server0='fastdds discovery --server-id 0 --ip-address $server_ip --port $server_port'
alias dds_server1='fastdds discovery --server-id 1 --ip-address $server_ip --port $robot_port1'
alias dds_server2='fastdds discovery --server-id 2 --ip-address $server_ip --port $robot_port2'
alias dds_server3='fastdds discovery --server-id 3 --ip-address $server_ip --port $robot_port3'

export TURTLEBOT3_MODEL=burger
export ROBOT_NAME=robot2
export MAP_NAME=maze
export ROS_DOMAIN_ID=30

source ~/turtlebot3_ws/install/local_setup.bash
source ~/turtlebot3_ws/install/setup.bash

source ~/Desktop/mobile_robot_ws/install/setup.bash
source ~/Desktop/mobile_robot_ws/install/local_setup.bash

#FASTRTPS_DEFAULT_PROFILES_FILE=super_client_cfg
