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
