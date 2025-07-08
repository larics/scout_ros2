# Marinero Localization Tmuxinator Session

This document describes the `marinero_localization` tmuxinator session defined in `session.yml`. This session launches all the necessary nodes for running the localization system on the Marinero robot.

## Overview

The session is named `marinero_localization` and is designed to start up the full software stack for robot localization. It is organized into several windows, each responsible for a specific part of the system (sensors, estimation, etc.). The `startup_window` is set to `estimation`, meaning it will be the active window upon session start.

## Pre-Window Setup

Before any windows are created, a `pre_window` script is executed to set up the environment:
-   `export AGV_NAMESPACE=marinero`: Sets the robot's namespace to `marinero`.
-   `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: Configures ROS 2 to use CycloneDDS.
-   `export ROS_DOMAIN_ID=10`: Sets the ROS domain ID to 10 for network communication.

## Windows

The session consists of the following windows:

### 1. `movella`

This window is responsible for launching the Movella sensor drivers.
-   **Pane 1**: Launches `movella_custom.launch.py`, which is the main driver for the Movella device (likely an Xsens IMU/GNSS).
-   **Pane 2**: Launches `ntrip_custom_launch.py`, an NTRIP client to receive RTK correction data for high-accuracy GPS positioning.
    -   **Note**: The user must manually create the NTRIP configuration file `custom_config/ntrip_config.yaml` based on the template `custom_config/ntrip_template_config.yaml`. The template contains placeholder values for the NTRIP server credentials that need to be replaced with actual values.

### 2. `mavros`

This window handles communication with the flight controller.
-   **Pane 1**: Launches MAVROS to connect to a PX4 flight controller on `/dev/ttyACM3`. It uses a custom `px4_config.yaml` file.

### 3. `estimation`

This is the core window for state estimation.
-   **Pane 1**: Launches the `estimator_manager.launch.py` from the `uav_ros_estimators` package. This node fuses data from various sensors to provide a robust state estimate for the robot.
    -   **Configuration**: The estimator is heavily configured using several YAML files:
        -   `horizontal_estimator_config.yaml`
        -   `heading_estimator_config.yaml`
        -   `height_estimator_config.yaml`
        -   `general_estimator_config.yaml`
    -   **Inputs**: It subscribes to numerous topics for sensor data, including IMU and GPS from both MAVROS and Movella, as well as odometry commands. All topic names are dynamically set using the `$AGV_NAMESPACE` variable.
    -   For a detailed description of the Estimator Manager, its configuration, and parameters, please refer to the [Estimator Manager Documentation](./estimator_manager.md).
-   **Pane 2**: Executes the `publish_home.sh` script, which likely publishes the robot's home position, a crucial piece of information for the estimators to establish a local coordinate frame.

### 4. `detections`

This window runs nodes related to dock detection and tracking.
-   **Pane 1**: `ros2 run dock_detector dockpeople2laserscan` - A node that may convert detections of people near a dock into a `LaserScan` message for obstacle avoidance.
-   **Pane 2**: `ros2 run dock_detector dock_detector` - The main node for detecting the docking station.
-   **Pane 3**: `ros2 run position_tracker position_tracker_tracking_node` - A node for tracking the position of the detected object (likely the dock).

### 5. `bag`

A utility window for data logging.
-   **Pane 1**: Changes the current directory to `~/bags`, preparing for the user to manually start or play `ros2 bag` files.
