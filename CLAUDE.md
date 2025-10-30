# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

CCAV-T (Cooperative & Connected AV Testing Application) is a ROS-based system for testing cooperative autonomous vehicle scenarios. It supports V2V (Vehicle-to-Vehicle) communication between ego and target vehicles through OBU (On-Board Unit) devices.

## ROS Environment Setup

This is a ROS catkin package. Before running any commands, set these environment variables:

**Primary Vehicle:**
```bash
export ROS_IP={IP_of_Primary}
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROS_HOSTNAME=$ROS_IP
```

**Secondary Vehicle:**
```bash
export ROS_MASTER_URI=http://{IP_of_Primary}:11311
export ROS_HOSTNAME={IP_of_Secondary}
```

## Building the Package

```bash
# From the catkin workspace root (typically ~/catkin_ws)
catkin_make
source devel/setup.bash
```

The build generates custom ROS messages defined in `msg/`:
- `ShareInfo.msg` - Main message containing vehicle state, pose, velocity, paths, and obstacles
- `Path.msg` - Path information for vehicle trajectories
- `Obstacle.msg` - Obstacle detection data

## Running the System

Launch scripts are in `manager/` directory. Always run from within the manager directory.

**Simulator Mode (Ego Vehicle):**
```bash
cd manager
./sim_ego.sh
```

**Real Vehicle Mode:**
```bash
cd manager
./ioniq5.sh  # For Ioniq5 ego vehicle
./avante.sh  # For Avante target vehicle
```

**Recording ROS Bags:**
```bash
cd manager
./bag.sh         # For ego vehicle
./bag_target.sh  # For target vehicle
```

**Kill All Processes:**
```bash
cd manager
./kill_all.sh
```

## Architecture

The system is organized into four main modules that communicate via ROS:

### 1. V2X Module (`v2x/`)
Handles V2V communication through OBU hardware using socket-based protocol.
- `main.py` - Entry point for V2X node
- `v2v_sharing.py` - V2VSharing class manages message exchange
- `ros_manager.py` - RosManager bridges ROS topics with V2X communication
- `libs/v2x_interface.py` - Hardware interface layer
- `libs/socket_handler.py` - Low-level socket communication

### 2. Self-Driving Module (`selfdriving/`)
Vehicle control and communication with physical vehicles or simulators.
- `main.py` - Entry point with SelfDriving class coordinating control and transmission
- `control/control.py` - Control class for path tracking (Pure Pursuit + PID)
- `transmitter/` - Vehicle-specific interfaces:
  - `simulator.py` - Simulated vehicle interface
  - `ioniq5.py` - Ioniq5 vehicle interface
  - `avante.py` - Avante vehicle interface

### 3. Sharing Info Module (`sharing_info/`)
Handles perception, planning, and HD map processing.
- `main.py` - Entry point with SharingInfo class
- `hd_map/` - HD map loading and lanelet processing
- `planning/` - Path planning components:
  - `local_path_planner.py` - LocalPathPlanner generates local paths
  - `velocity_planner.py` - VelocityPlanner determines speed profiles
- `perception/` - Obstacle detection and simulation:
  - `obstacle_handler.py` - ObstacleHandler processes obstacle data
  - `object_simulator.py` - Simulates obstacles for testing

### 4. UI Module (`ui/`)
PyQt-based user interface for system monitoring and control.
- `ui.py` - MyApp class provides the main Qt application
- `visualizer/visualizer.py` - RViz-based 3D visualization
- `ros_manager.py` - RosManager handles ROS topic subscriptions for UI updates

### Module Communication Flow

```
sharing_info → publishes ShareInfo messages
              → contains: state, signal, velocity, pose, paths, obstacles
                  ↓
v2x           → subscribes to ShareInfo
              → transmits over V2V to other vehicles
              → publishes received ShareInfo from other vehicles
                  ↓
selfdriving   → subscribes to ShareInfo (local and remote)
              → generates control commands
              → publishes to vehicle/simulator
```

## Key Architectural Patterns

1. **Vehicle Types**: System distinguishes between "ego" (controlled vehicle) and "target" (other vehicles)
2. **Map Support**: Multiple HD maps supported (Midan, KIAPI) specified via command-line arguments
3. **Mode Flexibility**: Same codebase runs in simulator or real vehicle mode
4. **RosManager Pattern**: Each module has a RosManager class that encapsulates all ROS communication

## Common Development Tasks

**Running a specific module independently:**
```bash
# V2X module (args: vehicle_type test_mode output_mode)
cd v2x && python3 main.py ego 0 out

# Self-driving module (args: vehicle_type car_type map_name)
cd selfdriving && python3 main.py ego simulator Midan

# Sharing info module (args: vehicle_type map_name test_mode)
cd sharing_info && python3 main.py ego Midan 0
```

**Testing utilities:**
```bash
# Calculate Time-To-Collision
cd utils && python3 calc_ttc.py

# Generate test data
cd utils && python3 make_data.py ego
```

## Custom Messages Location

Custom message definitions are in `msg/` and built via CMakeLists.txt. After modifying messages, rebuild with `catkin_make`.

## File Organization

- `manager/` - Shell scripts for launching different configurations
- `v2x/` - V2V communication layer
- `selfdriving/` - Vehicle control and hardware interfaces
- `sharing_info/` - Perception, planning, and mapping
- `ui/` - User interface and visualization
- `utils/` - Helper utilities and tools
- `msg/` - ROS custom message definitions
- `test/` - Test data and scenarios
- `log/` - Runtime logs
