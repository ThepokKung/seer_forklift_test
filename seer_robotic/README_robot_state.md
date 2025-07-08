# Robot State Node Usage Guide

## Overview

The Robot State Node is a ROS2 node that monitors robot battery level and current position by communicating with the robot's API endpoints. It publishes this information to ROS2 topics for use by other nodes.

## Features

- **Battery Monitoring**: Fetches battery voltage, percentage, current, and temperature
- **Position Tracking**: Retrieves current robot position (x, y, z coordinates)  
- **Configurable**: Uses YAML configuration files for robot and API settings
- **Multi-Robot Support**: Can monitor multiple robots with different namespaces
- **Logging**: Comprehensive logging with timestamps to file system
- **Error Handling**: Robust error handling and automatic reconnection

## API Endpoints Used

| Data Type | Port | API Number | Payload |
|-----------|------|------------|---------|
| Current Position | 19204 | 1004 | None (empty) |
| Battery State | 19204 | 1007 | None (empty) |

## Published Topics

### For Robot 1 (Default namespace):
- `/robot_position` (geometry_msgs/PointStamped) - Current robot position
- `/battery_state` (sensor_msgs/BatteryState) - Complete battery information
- `/battery_voltage` (std_msgs/Float32) - Battery voltage only
- `/battery_percentage` (std_msgs/Float32) - Battery percentage only

### For Robot 2 (With namespace):
- `/robot_02/robot_position` (geometry_msgs/PointStamped)
- `/robot_02/battery_state` (sensor_msgs/BatteryState)  
- `/robot_02/battery_voltage` (std_msgs/Float32)
- `/robot_02/battery_percentage` (std_msgs/Float32)

## Usage Examples

### 1. Single Robot Monitoring

#### Start Robot 1 monitoring:
```bash
ros2 launch seer_robotic robot_state.launch.py robot_key:=robot_01
```

#### Start Robot 2 monitoring with namespace:
```bash
ros2 launch seer_robotic robot_state.launch.py robot_key:=robot_02 robot_namespace:=robot_02
```

#### Custom update rate:
```bash
ros2 launch seer_robotic robot_state.launch.py robot_key:=robot_01 update_rate:=2.0
```

### 2. Multi-Robot Monitoring

#### Start both robots:
```bash
ros2 launch seer_robotic multi_robot_state.launch.py
```

#### Start with custom update rate:
```bash
ros2 launch seer_robotic multi_robot_state.launch.py update_rate:=0.5
```

#### Start only Robot 2:
```bash
ros2 launch seer_robotic multi_robot_state.launch.py enable_robot_01:=false
```

### 3. Manual Node Execution

#### Direct node execution:
```bash
ros2 run seer_robotic robot_state.py --ros-args -p robot_key:=robot_01 -p update_rate:=1.0
```

#### With namespace:
```bash
ros2 run seer_robotic robot_state.py --ros-args -r __ns:=/robot_02 -p robot_key:=robot_02 -p robot_namespace:=robot_02
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_key` | string | 'robot_01' | Robot configuration key from config file |
| `update_rate` | double | 1.0 | Update frequency in Hz |
| `robot_namespace` | string | '' | Namespace for published topics |

## Monitoring Topics

### View current position:
```bash
ros2 topic echo /robot_position
```

### View battery state:
```bash
ros2 topic echo /battery_state
```

### View battery percentage only:
```bash
ros2 topic echo /battery_percentage
```

### Monitor Robot 2 topics:
```bash
ros2 topic echo /robot_02/robot_position
ros2 topic echo /robot_02/battery_state
```

## Topic Details

### geometry_msgs/PointStamped (/robot_position)
```yaml
header:
  stamp: {sec: 1625234567, nanosec: 123456789}
  frame_id: "base_link"
point:
  x: 1.23
  y: 4.56
  z: 0.0
```

### sensor_msgs/BatteryState (/battery_state)
```yaml
header:
  stamp: {sec: 1625234567, nanosec: 123456789}
  frame_id: "battery"
voltage: 24.5
percentage: 0.85    # 85% (normalized 0-1)
current: 2.3
temperature: 25.0
power_supply_status: 2  # NOT_CHARGING
power_supply_health: 1  # GOOD
```

## Troubleshooting

### 1. Node fails to start
```bash
# Check robot configuration
ros2 run seer_robotic test_config.py

# Check network connectivity to robot
ping 192.168.0.180  # or 192.168.0.181
```

### 2. No topics published
```bash
# Check node status
ros2 node list
ros2 node info /robot_state

# Check logs
ros2 log echo robot_state
```

### 3. Connection errors
- Verify robot IP addresses in `config/robot_params.yaml`
- Check that robot API server is running on ports 19204
- Review log files in `scripts/logs/` directory

### 4. View detailed logs
```bash
# Check log files
ls seer_robotic/scripts/logs/
cat seer_robotic/scripts/logs/robot_api_$(date +%Y%m%d).log
```

## Configuration Files

### Robot Parameters (`config/robot_params.yaml`)
```yaml
robots:
  robot_01:
    robot_name: "SEER_Robot_01"
    robot_id: 1
    robot_ip: "192.168.0.180"
  robot_02:
    robot_name: "SEER_Robot_02" 
    robot_id: 2
    robot_ip: "192.168.0.181"
```

### API Ports (`config/api_ports.yaml`)
```yaml
api_ports:
  robot_status_api:
    port: 19204
    max_connections: 10
    category: "Robot Status API"
```

## Performance Notes

- **Update Rate**: Default 1 Hz is suitable for most applications
- **Network Load**: Each update makes 2 API calls (position + battery)
- **Error Handling**: Node automatically retries failed connections
- **Logging**: All operations logged with timestamps for debugging

## Integration with Other Nodes

### Example: Subscribe to position in another node
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            PointStamped,
            '/robot_position',
            self.position_callback,
            10)
    
    def position_callback(self, msg):
        self.get_logger().info(f'Robot at: x={msg.point.x}, y={msg.point.y}')
```

### Example: Monitor battery level
```python
from sensor_msgs.msg import BatteryState

def battery_callback(self, msg):
    percentage = msg.percentage * 100
    if percentage < 20:
        self.get_logger().warn(f'Low battery: {percentage:.1f}%')
```

## Launch File Options

All launch files support these common arguments:
- `update_rate`: Frequency of API calls (Hz)
- `robot_key`: Which robot configuration to use
- `robot_namespace`: ROS namespace for topics

Use `--show-args` to see all available options:
```bash
ros2 launch seer_robotic robot_state.launch.py --show-args
```
