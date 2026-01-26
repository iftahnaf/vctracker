# ROS2 Integration with micro-ROS

## Overview

The vctracker antenna tracker uses **micro-ROS** to receive target position data from ROS2. micro-ROS is a proper ROS2 implementation for microcontrollers that provides full compatibility with ROS2 topics, services, and actions.

### Why micro-ROS?

- ✅ **Native ROS2 messages** (sensor_msgs/NavSatFix)
- ✅ **Automatic serialization/deserialization**
- ✅ **Quality of Service (QoS) support**  
- ✅ **Works over USB serial** (no extra UART needed)
- ✅ **Standard ROS2 tools compatibility**
- ✅ **No custom bridge required**

## Architecture

```
ROS2 Host Computer                    Raspberry Pi Pico
┌───────────────────────┐            ┌────────────────────────┐
│                       │            │                        │
│  ROS2 Nodes/Topics    │            │  micro-ROS Client      │
│                       │            │                        │
│  /target/gps/fix   ───┼───USB──────┼───> Subscriber         │
│  (NavSatFix msg)      │   Serial   │      (NavSatFix)       │
│                       │            │                        │
│  micro-ROS Agent   <──┼────────────┼───  Pico SDK           │
│  (ros2 run...)        │            │                        │
│                       │            │                        │
└───────────────────────┘            └────────────────────────┘
```

## Prerequisites

### On Host Computer (Ubuntu 22.04/24.04)

```bash
# Install ROS2 (Humble or Jazzy)
sudo apt update
sudo apt install ros-humble-desktop

# Install micro-ROS agent
sudo apt install ros-humble-micro-ros-agent

# Source ROS2
source /opt/ros/humble/setup.bash
```

### On Pico (Build System)

The micro-ROS library is included as a submodule. Initialize it:

```bash
cd ~/vctracker
git submodule update --init --recursive
```

## Setup Instructions

### 1. Build Firmware with micro-ROS

```bash
# Clone micro-ROS SDK (if not already a submodule)
cd lib
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git

# Build the project
cd ~/vctracker
bash scripts/build.sh
```

The build process automatically:
- Links micro-ROS libraries
- Configures USB serial transport
- Sets up message types

### 2. Flash Firmware to Pico

```bash
# Put Pico in BOOTSEL mode
# Then flash
bash scripts/flash.sh
```

### 3. Start micro-ROS Agent on Host

Connect Pico via USB and start the agent:

```bash
# Find Pico USB device
ls /dev/ttyACM*
# Usually /dev/ttyACM0

# Start agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Expected output:
```
[1642424242.123456] info     | SerialAgentLinux.cpp | init    | running...
[1642424242.223456] info     | Root.cpp             | set_verbose_level | logger verbose level: 4
[1642424243.456789] info     | Root.cpp             | create_client | create | client_key: 0x12345678, session_id: 0x81
[1642424243.567890] info     | SessionManager.hpp   | establish_session | session established | client_key: 0x12345678, address: 0
```

### 4. Verify Connection

In another terminal, check for the Pico node:

```bash
# List nodes (should see /antenna_tracker)
ros2 node list

# Check topics (should see /target/gps/fix subscriber)
ros2 topic list

# Get node info
ros2 node info /antenna_tracker
```

## Publishing Target GPS Data

### Method 1: Command Line

```bash
ros2 topic pub /target/gps/fix sensor_msgs/msg/NavSatFix \
  "header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: 'gps'
  status:
    status: 0
    service: 1
  latitude: 37.7749
  longitude: -122.4194
  altitude: 150.5
  position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  position_covariance_type: 0"
```

### Method 2: Python Publisher Node

Create `target_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher = self.create_publisher(NavSatFix, '/target/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_gps)
        self.get_logger().info('Target GPS publisher started')
        
    def publish_gps(self):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        # Example: San Francisco coordinates
        msg.latitude = 37.7749
        msg.longitude = -122.4194
        msg.altitude = 150.5
        
        msg.status.status = 0  # Fix
        msg.status.service = 1  # GPS
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: lat={msg.latitude}, lon={msg.longitude}')

def main():
    rclpy.init()
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
chmod +x target_publisher.py
./target_publisher.py
```

### Method 3: Forward from Another GPS

Bridge another GPS source to the tracker:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSForwarder(Node):
    def __init__(self):
        super().__init__('gps_forwarder')
        
        # Subscribe to drone/vehicle GPS
        self.subscription = self.create_subscription(
            NavSatFix,
            '/drone/gps/fix',  # Source topic
            self.gps_callback,
            10
        )
        
        # Publish to tracker
        self.publisher = self.create_publisher(
            NavSatFix,
            '/target/gps/fix',  # Tracker topic
            10
        )
        
        self.get_logger().info('GPS forwarder running')
    
    def gps_callback(self, msg):
        # Forward message to tracker
        self.publisher.publish(msg)
        self.get_logger().debug('Forwarded GPS data')

def main():
    rclpy.init()
    node = GPSForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Monitoring

### View Pico Serial Output

```bash
# Using screen
screen /dev/ttyACM0 115200

# Using minicom
minicom -D /dev/ttyACM0 -b 115200
```

Expected output:
```
========================================
  Raspberry Pi Pico Antenna Tracker
  with micro-ROS Integration
========================================

Initializing micro-ROS...
Waiting for micro-ROS agent...
Connected to micro-ROS agent
Created node: antenna_tracker
Created subscriber for topic: /target/gps/fix
micro-ROS initialized successfully

--- Status Update ---
Tracker GPS: Lat=37.1234, Lon=-122.5678, Alt=50.0m, Sats=8, Fix=1
Target (ROS): Lat=37.7749, Lon=-122.4194, Alt=150.5m, Status=0
micro-ROS: Connected
Gimbal: Pan=25.3°, Tilt=5.2°
--------------------
```

### Monitor ROS2 Topics

```bash
# Check message rate
ros2 topic hz /target/gps/fix

# Echo messages
ros2 topic echo /target/gps/fix

# Show topic info
ros2 topic info /target/gps/fix
```

## Advanced Integration

### Launch File

Create `tracker_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start micro-ROS agent
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),
        
        # Start target GPS publisher
        Node(
            package='your_package',
            executable='target_publisher',
            name='target_publisher',
            output='screen'
        ),
    ])
```

Run:
```bash
ros2 launch your_package tracker_system.launch.py
```

### QoS Configuration

The tracker uses default QoS settings. To modify, edit `src/ROSParser.cpp`:

```cpp
// Change subscription QoS
rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;  // For sensor data
// or
rmw_qos_profile_t qos = rmw_qos_profile_default;      // Default reliable
```

### Multiple Trackers

Run multiple trackers with different node names:

1. Modify `main.cpp` to accept node name parameter
2. Or use ROS2 remapping:

```bash
# Terminal 1: Agent for Pico 1 on /dev/ttyACM0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# Terminal 2: Agent for Pico 2 on /dev/ttyACM1  
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1
```

## Troubleshooting

### Agent Not Connecting

**Problem**: `Failed to connect to micro-ROS agent`

**Solutions**:
1. Check USB connection
2. Verify agent is running: `ps aux | grep micro_ros_agent`
3. Check port: `ls /dev/ttyACM*`
4. Try different USB port
5. Check permissions: `sudo usermod -a -G dialout $USER` (logout/login)

### No Messages Received

**Problem**: ROS LED blinking fast, no target data

**Solutions**:
1. Verify topic name matches: `ros2 topic list`
2. Check message is being published: `ros2 topic hz /target/gps/fix`
3. Monitor Pico serial output for errors
4. Verify NavSatFix message format

### High Latency

**Problem**: Slow tracking updates

**Solutions**:
1. Check publishing rate: `ros2 topic hz /target/gps/fix`
2. Increase Pico update rate in `main.cpp`
3. Use sensor data QoS profile
4. Check USB cable quality

### Connection Drops

**Problem**: micro-ROS disconnects intermittently

**Solutions**:
1. Check USB cable and connection
2. Use powered USB hub
3. Verify power supply stability
4. Check agent logs for errors
5. Increase ping timeout in `ROSParser.cpp`

## Performance Optimization

### Reduce Latency

```cpp
// In ROSParser.cpp, reduce spin timeout
rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));  // 1ms instead of 10ms
```

### Reduce Bandwidth

Publish at lower rates:
```python
# In publisher node
self.timer = self.create_timer(0.5, self.publish_gps)  # 2 Hz instead of 10 Hz
```

### Memory Optimization

micro-ROS is optimized for constrained systems:
- Static memory allocation
- No dynamic memory after initialization
- Configurable history depth

## ROS2 Package Integration

### Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python tracker_nodes \
    --dependencies rclpy sensor_msgs

cd tracker_nodes/tracker_nodes
# Add your publisher/forwarder nodes here
```

### Build and Install

```bash
cd ~/ros2_ws
colcon build --packages-select tracker_nodes
source install/setup.bash
```

## References

- [micro-ROS Documentation](https://micro.ros.org/)
- [micro-ROS for Pico](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)
- [Getting Started Guide](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico)
- [RobotFoundry Tutorial](https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3)
- [sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)

## Next Steps

- [Hardware Setup](hardware.md)
- [Software Setup](setup.md)
- Test with simulated GPS data
- Deploy in real tracking scenario
- Add custom ROS2 services for configuration
