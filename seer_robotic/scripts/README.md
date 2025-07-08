# Robot API Communication Scripts

This directory contains Python scripts for communicating with the robot server using a custom protocol, with comprehensive configuration management and logging systems.

## Files Overview

### Configuration Files (`../config/`)
- **`robot_params.yaml`** - Robot parameters (names, IDs, IP addresses)
- **`api_ports.yaml`** - API port configurations and categories

### Backend Package (`backend/`)
- **`api.py`** - Main API client class for robot communication
- **`packker.py`** - Message packing/unpacking utilities
- **`config_manager.py`** - Configuration management system
- **`logger.py`** - Logging system with file and console output
- **`__init__.py`** - Package initialization

### Example Scripts
- **`config_example.py`** - Comprehensive examples using configuration system
- **`test_config.py`** - Configuration testing without robot connection
- **`test_logging.py`** - Logging system testing
- **`example_usage.py`** - Basic usage examples
- **`standalone_robot_api.py`** - Standalone version (replicates original code)

### Support Files
- **`requirements.txt`** - Python dependencies
- **`README.md`** - This documentation

### Log Files (`logs/`)
- **`robot_api_YYYYMMDD.log`** - Daily log files with timestamps
- Automatic creation when logging is used

## Logging System

### Features
- **Timestamped logs** - All log entries include date and time
- **Multiple log levels** - DEBUG, INFO, WARNING, ERROR, CRITICAL
- **File and console output** - Logs to files and optionally to console
- **Hex data logging** - Special formatting for binary data
- **JSON data logging** - Pretty-printed JSON in logs
- **Daily log rotation** - New log file each day

### Log Levels
- **DEBUG** - Detailed information for debugging
- **INFO** - General information about operation
- **WARNING** - Warning messages (also shown on console)
- **ERROR** - Error messages (also shown on console)
- **CRITICAL** - Critical errors (also shown on console)

### Usage
```python
from backend.logger import get_logger

logger = get_logger("my_component")
logger.info("Operation completed successfully")
logger.error("An error occurred")
logger.log_hex_data(binary_data, "Network packet")
logger.log_json_data(json_data, "API response")
```

## Configuration System

### Robot Configuration (`robot_params.yaml`)
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

### API Configuration (`api_ports.yaml`)
```yaml
api_ports:
  robot_status_api:
    port: 19204
    max_connections: 10
    category: "Robot Status API"
  robot_control_api:
    port: 19205
    max_connections: 5
    category: "Robot Control API"
  # ... more APIs
```

## Usage

### Method 1: Using Configuration System (Recommended)

```python
from backend.api import RobotAPI

# Use default robot and API
api = RobotAPI()

# Use specific robot
api = RobotAPI(robot_key='robot_02')

# Use specific API
api = RobotAPI(api_name='robot_control_api')

# Use specific robot and API
api = RobotAPI(robot_key='robot_01', api_name='robot_navigation_api')
```

### Method 2: Direct Configuration

```python
# Override with direct parameters
api = RobotAPI(ip='192.168.0.180', port=19204)
```

### Method 3: Context Manager

```python
with RobotAPI(robot_key='robot_02', api_name='robot_status_api') as api:
    response = api.send_api_request(
        api_number=1004,
        payload={"query": "status"},
        req_id=1
    )
```

## API Port Categories

| Category | Port | Max Connections | Description |
|----------|------|-----------------|-------------|
| Robot Status API | 19204 | 10 | Robot status information |
| Robot Control API | 19205 | 5 | Robot movement control |
| Robot Navigation API | 19206 | 5 | Navigation and path planning |
| Robot Configuration API | 19207 | 5 | Robot configuration management |
| Other API | 19210 | 5 | General purpose operations |
| Robot Push API | 19301 | 10 | Real-time notifications |

## Installation

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Test configuration**:
   ```bash
   python3 test_config.py
   ```

3. **Test logging**:
   ```bash
   python3 test_logging.py
   ```

## Running Examples

1. **Configuration Examples**:
   ```bash
   python3 config_example.py
   ```

2. **Basic Examples**:
   ```bash
   python3 example_usage.py
   ```

3. **Standalone Version**:
   ```bash
   python3 standalone_robot_api.py
   ```

4. **Test Scripts**:
   ```bash
   python3 test_config.py
   python3 test_logging.py
   ```

## Log File Structure

Log files are automatically created in the `logs/` directory:

```
logs/
├── robot_api_20250704.log
├── robot_api_20250705.log
└── ...
```

### Log Format
```
2025-07-04 14:30:25 - robot_api - INFO - Robot API initialized - IP: 192.168.0.180, Port: 19204, Timeout: 5s
2025-07-04 14:30:25 - robot_api - INFO - Connection established: 192.168.0.180:19204
2025-07-04 14:30:25 - robot_api - INFO - API Request - Number: 1004, ReqID: 1
2025-07-04 14:30:25 - robot_api - DEBUG - Sending request: 5A 01 00 01 00 00 00 00 03 EC 00 00 00 00 00 00
```

## Configuration Management

### Access Robot Information
```python
from backend.config_manager import ConfigManager

config = ConfigManager()

# Get robot information
robot_ip = config.get_robot_ip('robot_01')
robot_id = config.get_robot_id('robot_01')
robot_name = config.get_robot_name('robot_01')

# List all robots
robots = config.list_robots()
```

### Access API Information
```python
# Get API port
port = config.get_api_port('robot_status_api')

# Get API by category
api_name = config.get_api_by_category('status')

# List all APIs
apis = config.list_apis()
```

## Protocol Details

The communication protocol uses the following structure:

### Header Format (16 bytes)
- **Start Byte** (1 byte): `0x5A`
- **Version** (1 byte): `0x01`
- **Request ID** (2 bytes): Unique request identifier
- **Message Length** (4 bytes): Length of JSON payload
- **Message Type** (2 bytes): API number
- **Reserved** (6 bytes): `0x00` padding

### Message Flow
1. Pack header with message information
2. Append JSON payload (if any)
3. Send complete message to server
4. Receive response header (16 bytes)
5. Receive JSON response data (if length > 0)

## Error Handling

The scripts include comprehensive error handling for:
- Connection failures
- Socket timeouts
- Invalid response headers
- JSON parsing errors
- Protocol violations
- Configuration loading errors

All errors are logged with timestamps and context information.

## Multi-Robot Support

The configuration system supports multiple robots:

```python
# Communicate with all robots
config = ConfigManager()
for robot_key in config.list_robots():
    api = RobotAPI(robot_key=robot_key)
    # ... communicate with robot
```

## ROS2 Integration

These scripts are designed to work within a ROS2 package structure and can be easily integrated into ROS2 nodes for robot control and monitoring.

## Troubleshooting

1. **Configuration not loading**: Check YAML file syntax
2. **Import errors**: Install PyYAML with `pip install PyYAML`
3. **Connection failures**: Verify robot IP addresses and ports
4. **API errors**: Check API port configuration matches robot server
5. **No log files**: Check that the `logs/` directory has write permissions
6. **Large log files**: Logs rotate daily, old files can be archived or deleted

## Log Management

### Log Location
Logs are stored in `scripts/logs/` directory with daily rotation:
- `robot_api_20250704.log` - Today's log
- `robot_api_20250703.log` - Yesterday's log

### Log Cleanup
You can safely delete old log files to save space:
```bash
# Remove logs older than 7 days
find logs/ -name "*.log" -mtime +7 -delete
```
