#!/bin/bash

# Example script showing how to call the assign_task service using ros2 service call

echo "=== Robot Controller Assign Task Service Examples ==="
echo ""

echo "Task Type IDs:"
echo "1 = PickInit"
echo "2 = PlaceInit" 
echo "3 = PickToManipulator"
echo "4 = PickFromManipulator"
echo ""

echo "=== Example 1: PickInit (task_type_id=1) ==="
echo "ros2 service call /robot_controller/assign_task seer_robot_interfaces/srv/AssignTask \"{task_type_id: 1, pallet_id: 101, task_id: 'task_pick_init_001'}\""
echo ""

echo "=== Example 2: PlaceInit (task_type_id=2) ==="
echo "ros2 service call /robot_controller/assign_task seer_robot_interfaces/srv/AssignTask \"{task_type_id: 2, pallet_id: 102, task_id: 'task_place_init_002'}\""
echo ""

echo "=== Example 3: PickToManipulator (task_type_id=3) ==="
echo "ros2 service call /robot_controller/assign_task seer_robot_interfaces/srv/AssignTask \"{task_type_id: 3, pallet_id: 103, task_id: 'task_pick_to_manip_003'}\""
echo ""

echo "=== Example 4: PickFromManipulator (task_type_id=4) ==="
echo "ros2 service call /robot_controller/assign_task seer_robot_interfaces/srv/AssignTask \"{task_type_id: 4, pallet_id: 104, task_id: 'task_pick_from_manip_004'}\""
echo ""

echo "=== Example 5: Invalid task_type_id (should fail) ==="
echo "ros2 service call /robot_controller/assign_task seer_robot_interfaces/srv/AssignTask \"{task_type_id: 99, pallet_id: 105, task_id: 'task_invalid_099'}\""
echo ""

# Uncomment the lines below to actually execute the service calls
# (Make sure the robot_controller node is running first)

# echo "Executing PickInit example..."
# ros2 service call /robot_controller/assign_task seer_robot_interfaces/srv/AssignTask "{task_type_id: 1, pallet_id: 101, task_id: 'task_pick_init_001'}"
