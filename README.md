# LASER USV PX4 API

This API serves as the communication bridge between the autopilot hardware and the high/low-level control systems on the companion computer, managing time synchronization and reference frame conversions.

-   **Subscribed Topics:**
    - `/rahcm/actuator_motors`: Used to send normalized commands directly to USV's motors.

-   **Published Topics:**
    - `/imu`: Publishes filtered data from the autopilot's internal IMU (Inertial Measurement Unit).
    - `/odometry`: Publishes the estimated state of the USV, including its position, orientation, and velocities.

-   **Services:**
    - `/arm`: Service to arm the USV's motors. 
    - `/disarm`: Service to disarm the USV's motors. 

-   The Offboard mode configuration can be set in `api.yaml`:
```
offboard: 
    position: true              # to work with control manager
    velocity: false
    acceleration: false
    attitude: false
    thrust_and_torque: false
    body_rate: false
    direct_actuator: false      # to work with teleop
```
This configuration defines which setpoint type PX4 will accept through Offboard mode. The fields are prioritized from top to bottom, so only one control level should be enabled at a time. To use the teleoperation node (from [`laser_usv_teleop`](https://github.com/LASER-Robotics/laser_usv_teleop)), set `direct_actuator: true`. To use the Control Manager (from [`laser_usv_control_manager`](https://github.com/LASER-Robotics/laser_usv_control_manager)), set `position: true` and keep all other fields set to `false`.
