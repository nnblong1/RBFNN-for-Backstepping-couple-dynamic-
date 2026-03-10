1. 
MicroXRCEAgent udp4 -p 8888

2.
ros2 launch uam_controller uam_system.launch.py sim:=true

3.
ros2 run uam_controller lstm_feedforward_node.py

4.
# Terminal khác: ARM + Offboard
sleep 3
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{command: 176, param1: 1.0, param2: 6.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
sleep 1
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{command: 400, param1: 1.0, param2: 21196.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"