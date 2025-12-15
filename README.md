# Robot Sensor Setup & Visualization Guide

Date: December 15, 2025
OS: Raspberry Pi (Linux)

## 1. Prerequisites
- **SSH with X11 Forwarding:** You must connect to the robot using the `-X` flag to view the UI.
  ```bash
  ssh -X ronaldo@<robot_ip>
  ```
- **ROS2 Environment:** Ensure your environment is sourced (usually done automatically by `.bashrc`).
  ```bash
  source ~/.bashrc
  # or manually:
  # source /opt/ros/jazzy/setup.bash
  # source ~/workspace/install/setup.bash
  ```

## 2. Starting the Sensors

You will need two separate terminal windows.

### Terminal 1: YDLIDAR (Native)
The Lidar driver runs natively on the host system (ROS2 Jazzy).
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```
*   **Frame ID:** `laser_frame`
*   **Topic:** `/scan`

### Terminal 2: Astra Depth Camera (Docker)
The depth camera runs inside a Docker container (ROS2 Humble) with host networking to communicate with the host.
```bash
cd ~/tmp/docker3
docker compose up
```
*   **Frame ID:** `camera_link` (or `camera_color_optical_frame`)
*   **Topics:** `/camera/color/image_raw`, `/camera/depth/points`, etc.
*   *Note:* The container is configured to use the host's network and `wlan0` interface via `cyclonedds.xml`.

### Rebuilding the Camera Image
**Correct Source:** `~/tmp/docker3` (Matches `astra-ros2:humble-1.2`)
*Note: `~/tmp/docker2` contains an older version without CycloneDDS support.*

```bash
cd ~/tmp/docker3
docker build -t astra-ros2:humble-1.2 .
```

## 3. Visualization

### Option A: Camera Feeds Only (Simple)
Best for checking if the camera is working.
```bash
ros2 run rqt_image_view rqt_image_view
```
*   Select the topic from the top-left dropdown (e.g., `/camera/color/image_raw`).

### Option B: Lidar & 3D Depth (RViz2)
Best for checking spatial data and alignment.
```bash
rviz2
```

#### Troubleshooting Visualization

1.  **Lidar Data Not Showing (/scan):**
    *   **Symptom:** "New publisher discovered ... incompatible QoS".
    *   **Fix:** In RViz, expand the **LaserScan** properties -> **Topic** -> **QoS**. Change **Reliability** to **Best Effort**.

2.  **Frames & Transforms (TF):**
    *   **Issue:** You cannot see the Lidar and Camera data at the same time because they are not connected by a transform (TF).
    *   **To see Lidar:** Set **Fixed Frame** (Global Options) to `laser_frame`.
    *   **To see Camera (PointCloud):** Set **Fixed Frame** to `camera_link`.
    *   **To see Both:** You must publish a static transform to link them.
        ```bash
        # Example: Link laser_frame to camera_link (adjust x y z yaw pitch roll as needed)
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 laser_frame camera_link
        ```

## 4. System Notes
- **Docker Configuration:** The `docker-compose.yml` in `~/tmp/docker3` mounts a local `cyclonedds.xml` to configure the middleware.
- **Kernel Settings:** `net.core.rmem_max` was increased to `16777216` to support the camera's high bandwidth.
