## Estimator Manager Launch File

This document describes the `estimator_manager.launch.py` file, which is responsible for launching the Estimator Manager node.

### Overview

This launch file starts the Estimator Manager, a sophisticated ROS 2 node that fuses data from various sensors to provide a robust state estimate (position, velocity, orientation) for the robot. It is designed as a composable node and is loaded into a component container.

The manager is composed of three main estimators:
1.  **Height Estimator**: Fuses sensor data to estimate vertical position, velocity, and acceleration.
2.  **Horizontal Estimator**: Fuses sensor data to estimate planar position, velocity, and acceleration.
3.  **Heading Estimator**: Fuses sensor data to estimate heading and heading rate.

Each estimator is a separate Kalman Filter that can be configured to use different sensor measurements and process models via YAML configuration files. This launch file exposes arguments to specify these configuration files and to remap the input sensor topics.

### Launch Arguments

#### Estimator Configuration
-   `general_config`: Path to a general YAML configuration file for shared parameters.
-   `heading_config`: Path to the YAML configuration file for the heading estimator.
-   `height_config`: Path to the YAML configuration file for the height estimator.
-   `horizontal_config`: Path to the YAML configuration file for the horizontal estimator.
-   `namespace`: The namespace for the robot. Defaults to the `UAV_NAMESPACE` environment variable.
-   `use_sim_time`: Boolean flag to indicate whether to use simulation time. Defaults to `"False"`.
------------------------------------
#### Input Topics

-   `ar_tag_topic`: Input topic for AprilTag-based position measurements (`geometry_msgs/msg/PoseStamped`).
    -   Used for horizontal position correction.
    -   **Config Key**: `ar_tag`
-   `cmd_vel_topic`: Input topic for velocity commands (`geometry_msgs/msg/Twist`).
    -   Used as an input for the prediction step (model update) of the Kalman filters.
    -   **Config Key**: `cmd_vel`
-   `magfield_topic`: Input topic for magnetometer data (`sensor_msgs/msg/MagneticField`).
    -   Used for heading correction based on magnetic field measurements.
    -   **Config Key**: `mag_heading`
-   `mavros_gps_raw_fix_topic`: Input topic for raw GPS fix data (`sensor_msgs/msg/NavSatFix`).
    -   Used after conversion to the local frame for horizontal position correction.
    -   **Config Key**: `gps_raw_fix`
-   `mavros_gps_raw_vel_topic`: Input topic for raw GPS velocity data (`geometry_msgs/msg/TwistStamped`).
    -   Used for horizontal velocity correction.
    -   **Config Key**: `gps_raw_vel`
-   `mavros_home_topic`: The topic for the MAVROS home position.
    -   Used by the `GlobalToLocal` utility to set the origin of the local coordinate frame.
    -   **Config Key**: N/A
-   `mavros_imu_topic`: Input topic for the primary IMU data from MAVROS (`sensor_msgs/msg/Imu`).
    -   Used as a core sensor for gravity estimation, heading, and acceleration measurements for all three estimators.
    -   **Config Keys**: `imu_acc` (horizontal, height), `imu_hdg` (heading), `imu_vel` (heading velocity)
-   `mavros_odom_topic`: Input topic for odometry data from MAVROS (`nav_msgs/msg/Odometry`).
    -   Used to provide position and velocity corrections for both horizontal and height estimators.
    -   **Config Keys**: `mavros_pos` (horizontal, height), `mavros_vel` (horizontal, height)
-   `movella_gps_raw_fix_topic`: Input topic for raw GPS fix data from a secondary (Movella) GPS (`sensor_msgs/msg/NavSatFix`).
    -   Used for horizontal corrections after the conversion to the local frame.
    -   **Config Key**: `movella_raw_fix` (and `zupt` if ZUPT is enabled)
-   `movella_gps_vector3_topic`: Input topic for raw GPS fix data from a secondary (Movella) GPS, provided as a Vector3 (`geometry_msgs/msg/Vector3Stamped`).
    -   Used for horizontal position correction.
    -   **Config Key**: `movella_raw_fix`
-   `movella_imu_topic`: Input topic for a secondary (Movella) IMU (`sensor_msgs/msg/Imu`).
    -   Used as a sensor for heading, height, and horizontal correction.
    -   **Config Keys**: `movella_imu_acc` (horizontal, height), `movella_imu_hdg` (heading), `movella_imu_vel` (heading)
-   `odom_cmd_vel_in_topic`: Input topic for velocity commands from an odometry message (`nav_msgs/msg/Odometry`).
    -   Used as an alternative source for the prediction step (model update) of the Kalman filters.
    -   **Config Key**: `cmd_vel`
------------------------------------
#### Output Topics

-   `main/odometry`: The primary output of the estimator. It publishes the fused odometry message (`nav_msgs/msg/Odometry`) containing the estimated position, orientation, and velocities.
-   `estimators_info`: Publishes detailed information about the status of all configured estimators (`uav_ros_msgs/msg/UAVEstimatorsInfo`), including which estimators are active and the health of their sensor inputs.
-   `debug/est/<estimator_frame_id>`: A series of topics that publish the full odometry output (`nav_msgs/msg/Odometry`) for every combination of estimators defined in the configuration, not just the active one. This is useful for debugging and comparing different estimator configurations.

### Configuration

The Estimator Manager is highly configurable through YAML files. The launch file accepts paths to four separate configuration files: `general_config`, `horizontal_config`, `height_config`, and `heading_config`. These files control everything from the filter's process noise to which sensor measurements are fused by each estimator.

The core of the configuration revolves around defining a set of estimators and specifying which `measurements` and `inputs` each one will use.

-   **Estimators**: These are the individual Kalman Filter instances. You can define multiple estimators with different configurations within a single file (e.g., one that fuses GPS and another that fuses odometry).
-   **Measurements**: These are sensor data topics that are used in the "correction" or "update" step of the Kalman filter (e.g., `lora_pos`, `baro_pos`, `imu_acc`). For each measurement, you must define its covariance matrix `R`, which represents the measurement noise.
-   `model_type`: (string) Selects the physical motion model used for the filter's prediction step. This choice dictates the structure of the state transition matrix (\(A\)) which predicts how the state evolves over time. For example:
    -   `CONST_VEL` (Constant Velocity): Assumes the robot moves with a near-constant velocity. The state typically includes position and velocity. This model is simpler and works well when the system dynamics are smooth.
    -   `CONST_ACC` (Constant Acceleration): Assumes the robot moves with a near-constant acceleration. The state includes position, velocity, and acceleration. This model can react more quickly to changes in velocity.
-   `model_a_coefficients`, `model_b_coefficients`: (array of double) These are tuning parameters for the state transition matrix (\(A\)) and the control-input matrix (\(B\)), respectively. They allow for fine-tuning the physics of the motion model (e.g., adding damping, scaling inputs) without changing the source code.
-   `Q`: (array of double) This is the **process noise covariance matrix**. It represents the uncertainty in the `model_type`. In the filter's prediction step, the state covariance is inflated by \(Q\) to reflect that the model is not perfect.
    -   **High \(Q\) values**: Indicate low confidence in the motion model's predictions, causing the filter to rely more heavily on incoming sensor measurements. This makes the estimate more responsive but also more susceptible to measurement noise.
    -   **Low \(Q\) values**: Indicate high confidence in the model, resulting in a smoother estimate that is less affected by noisy measurements. However, it may be slower to react to actual changes in motion.
-   `estimators`: (array of string) A list of all available estimators that can be defined and switched between.
-   `measurements`: (array of string) A list of all possible sensor measurements that can be used by any of the estimators.
-   `inputs`: (array of string) A list of all possible command inputs for the prediction step.
-   `[ESTIMATOR_NAME]`: A block that defines a specific estimator by specifying its `fused_measurements` and `inputs`.
-   `[MEASUREMENT_NAME]`: A block that defines a specific measurement's properties, such as its `type`, measurement noise `R`, and `timeout`.
-   `valid_estimators`: This is a critical section that defines the valid *combinations* of estimators for which TF frames are published.

---

#### A and B Matrix Examples

---
##### **Constant Velocity Model Example**

To better understand how the model coefficients work, let's examine the `CONSTANT_ACCELERATION` model defined in `include/uav_ros_estimators/estimators/lkf_horizontal.hpp`. The state vector `x` being estimated is:
```
x = [px, vx, ax, py, vy, ay]^T
```
where p, v, and a are position, velocity, and acceleration, respectively.

The filter's prediction step is governed by the equation:
```
x_k = A * x_{k-1} + B * u_{k-1}
```


**A Matrix (State Transition)**

With `model_a_coefficients: [0.1, 0.1]`, the A matrix applies damping directly to the velocity states:
```
A = | 1    dt    0    0      0      0 |
    | 0    1-0.1 0    0      0      0 |
    | 0    0     0    0      0      0 |
    | 0    0     0    1      dt     0 |
    | 0    0     0    0      1-0.1  0 |
    | 0    0     0    0      0      0 |
```

The parameters act as a damping factor on the acceleration state, assuming that the acceleration will not be perfectly constant between time steps.

**B Matrix (Control Input)**

With `model_b_coefficients: [1.0, 1.0]` and a velocity input command (e.g., from `cmd_vel`), the B matrix applies the input directly to the velocity states:
```
B = | 0.0  0.0 |
    | 1.0  0.0 |
    | 0.0  0.0 |
    | 0.0  0.0 |
    | 0.0  1.0 |
    | 0.0  0.0 |
```
This shows how an external velocity command would directly influence the estimated velocity.

By tuning these coefficients, you can adjust the filter's behavior to more closely match the real-world physics of your robot without needing to recompile the code.

#### `horizontal_estimator_config.yaml`

This file configures the estimators responsible for tracking the robot's state in the X-Y plane.

-   `active_estimator`: (string) The name of the horizontal estimator to use on startup.
-   `model_type`: (string) The process model to be used by the Kalman filter (e.g., `CONST_ACC`, `CONST_VEL`). This determines the structure of the state transition matrix `A` and the control input matrix `B`.
-   `model_a_coefficients`, `model_b_coefficients`: (array of double) Coefficients for the process model matrices.
-   `Q`: (array of double) The process noise covariance matrix. It represents the uncertainty in the process model.
-   `estimators`: (array of string) A list of all available horizontal estimators that can be defined and switched between.
-   `measurements`: (array of string) A list of all possible sensor measurements that can be used by any of the horizontal estimators.
-   `inputs`: (array of string) A list of all possible command inputs for the prediction step.
-   `[ESTIMATOR_NAME]`: A block that defines a specific estimator.
    -   `fused_measurements`: An array of strings specifying which measurements from the main list this particular estimator should fuse.
    -   `inputs`: An array of strings specifying which inputs this estimator should use.
-   `[MEASUREMENT_NAME]`: A block that defines a specific measurement.
    -   `type`: The type of measurement (e.g., `POS`, `VEL`), which determines the structure of the observation matrix `H`.
    -   `R`: The measurement noise covariance matrix for this sensor.
    -   `timeout`: The time in seconds after which a sensor is considered unresponsive.

---

#### `height_estimator_config.yaml` & `heading_estimator_config.yaml`

These files follow the exact same structure as the `horizontal_estimator_config.yaml` but are for the vertical (Z) and heading (yaw) dimensions, respectively.

-   **`height_estimator_config.yaml`**: Configures estimators that fuse measurements like barometer position (`baro_pos`), IMU vertical acceleration (`imu_acc`), and MAVROS odometry (`mavros_pos`). It uses a `CONST_ACC` model.
-   **`heading_estimator_config.yaml`**: Configures estimators that fuse measurements like IMU heading and rate (`imu_hdg`, `imu_vel`) and magnetometer data (`mag_heading`). It uses a `CONST_VEL` model.

---

#### `general_estimator_config.yaml`

This file contains parameters that are shared across all estimators.

```yaml
general_estimator:
  gps_fix_throttle_period: 0.0
  gps_fix_delay: 0.0
  gps_fix_mean: 0.0
  gps_fix_stddev: 0.0
  R_variable: false
  valid_estimators:
    height: ["BARO", "BARO", "BARO", "MAVROS", "MAVROS"]
    heading: ["IMU", "IMU_FW", "MOVELLA", "MAVROS", "MAVROS"]
    horizontal: ["GT", "LORA", "MOVELLA", "MAVROS", "AR_TAG"]
  agv_specific:
    gps_R_hor_moving: [1.0, 0.0,
          0.0, 1.0]
    gps_R_hor_stationary: [100000.0, 0.0,
          0.0, 100000.0]
    rotating_R_scaling: 1.0
    enable_zupt: true
```

-   `gps_fix_throttle_period`, `gps_fix_delay`, `gps_fix_mean`, `gps_fix_stddev`: Parameters used in `mavros_gps_fix_cb` to control the rate and add simulated delay/noise to GPS measurements for testing.
-   `R_variable`: A boolean flag that, if true, allows the measurement covariance `R` for the LoRa position to be updated dynamically from the incoming message (`lora_pose_cb`).
-   `valid_estimators`: This is a critical section that defines the valid *combinations* of estimators that can be published as transforms.
-   `agv_specific`: A namespace for parameters that enable an adaptive Kalman filter tuning, particularly for ground robots.
    -   `gps_R_hor_moving` / `gps_R_hor_stationary`: These allow for dynamically changing the GPS measurement covariance (`R` matrix) based on whether the robot is moving or not. This is useful because GPS drift is more significant when the robot is stationary.
    -   `rotating_R_scaling`: A multiplier applied to the measurement covariance `R` for the heading estimator when the robot is rotating. This can be used to temporarily distrust the heading measurement during high-rate turns.
    -   `enable_zupt`: A boolean to enable Zero-velocity UPdaTes (ZUPT). When the robot is stationary, this feature provides a zero-velocity measurement to the filter, which helps to mitigate position and velocity drift.