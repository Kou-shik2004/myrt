# myrt

myrt is an in-progress ROS 2 differential-drive robot workspace, built as a group project with sukesh and Girish Raghav. The workspace covers a CAD-exported robot description simulated in Gazebo Classic, a stock `ros2_control` differential-drive configuration, a serial bridge to an Arduino, and a custom OpenCV color-blob detection pipeline with a purpose-built message type. There is no navigation stack: no Nav2, no SLAM, no localization, and no MoveIt. The RViz configuration includes the stock "2D Pose Estimate" and "2D Goal Pose" tools, which are part of the default RViz tool set and are not backed by any planner or localization node in this repository.

## Package layout

The workspace is a ROS 2 `src/` directory of five sibling packages, split by concern:

| Package | Contents |
|---|---|
| `custom_msgs` | Message definitions for the vision pipeline: `Point`, `Contour`, `ImagePlusTupleList` |
| `myrt_description` | URDF (via xacro), meshes, Gazebo models, and RViz configs |
| `myrt_controller` | `ros2_control` controller parameters and a spawner launch file |
| `myrt_firmware` | Serial bridge nodes between ROS 2 and the Arduino sketches |
| `sample_pkg` | Publisher/subscriber demo nodes and the color-blob detection pipeline |

## Requirements

- Ubuntu 22.04 and ROS 2 Humble are the best inference from the workspace's contents (Gazebo Classic dependencies and `cpython-310` compiled artifacts), but no README, Dockerfile, or CI configuration in the workspace states the distro explicitly. Confirm before relying on this.
- Gazebo Classic 11 (via `gazebo_ros_pkgs`, `gazebo_ros2_control`)
- Python 3.10, OpenCV, NumPy
- An Arduino board for `myrt_firmware`, if using the serial bridge

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-rviz2 \
  ros-humble-cv-bridge python3-opencv python3-numpy python3-serial
```

## Building

```bash
mkdir -p ~/myrt_ws/src
cd ~/myrt_ws/src
git clone git@github.com:Kou-shik2004/myrt.git .

cd ~/myrt_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

This has not been built or run on ROS 2 Humble on the machine used to write this document (Jazzy only). Treat the build and launch steps below as unverified until confirmed in a Humble environment.

## Running it

### View the robot in RViz

```bash
ros2 launch myrt_description display.launch.py
```

Publishes robot state from the xacro-expanded URDF and opens RViz with `joint_state_publisher_gui` for manual joint control. No simulator involved.

### Simulate in Gazebo

```bash
ros2 launch myrt_description gazebo.launch.py
```

Starts Gazebo Classic, publishes robot state, and spawns the robot from `robot_description`. `myrt_description/launch/view.launch.py` is a near-duplicate of this launch file that spawns the robot under a different entity name; it is not a separate workflow and can be ignored.

### Bring up the controllers

```bash
ros2 launch myrt_controller controller.launch.py
```

Spawns the `joint_state_broadcaster` and `diff_base_controller` against `/controller_manager`. This launch file does not start a controller manager itself; it depends on the one embedded by `gazebo_ros2_control` when `gazebo.launch.py` is running. Run `gazebo.launch.py` first.

### Serial bridge

```bash
ros2 run myrt_firmware simple_serial_receiver.py
ros2 run myrt_firmware simple_serial_transmitter.py
```

Each node takes `port` (default `/dev/ttyACM0`) and `baudrate` (default `115200`) parameters and requires the matching Arduino sketch under `myrt_firmware/firmware/` to be flashed to the board.

## The vision pipeline

`sample_pkg` includes a color-blob detection pipeline built around a custom message type:

```bash
ros2 run sample_pkg cam_node        # captures, detects, and publishes
ros2 run sample_pkg cam_view_node   # subscribes and draws the result
```

`cam_node` (`sample_pkg/video_publisher.py`) loads an offline camera calibration, undistorts each frame, applies hand-tuned HSV thresholds for red, green, blue, and yellow, and extracts contours per color with OpenCV's `findContours`. The compressed frame and per-color contour lists are packed into `custom_msgs/ImagePlusTupleList` and published on `/rpi_video_feed`, so the heavier per-frame work stays on the publishing side and a viewer only needs to draw. `cam_view_node` (`sample_pkg/video_subscriber.py`) subscribes, reconstructs each contour, and draws a labeled bounding box per detection.

The detection technique itself is standard OpenCV color thresholding; the custom part is the message design and the threshold tuning, not a novel algorithm.

## Known limitations

- `sample_pkg`'s `video_publisher.py` constructs the outgoing message inside its per-color loop but publishes outside it, so only the last color processed on a given frame (yellow, by iteration order) is ever published; detections for the other three colors are computed and discarded.
- In the same file, each contour's points are appended to the message once per point rather than once per contour, so a contour with many points is duplicated that many times in the published message. This is also why the topic's queue depth was raised to 1000 in an earlier commit rather than fixing the underlying loop.
- Bounding boxes are drawn against the raw, distorted frame, while the contours used to compute them were detected on the undistorted frame, so boxes can be slightly offset from the objects they mark.
- `video_publisher.py` loads its calibration file from a hardcoded path (`/home/pi/myrt_ws/src/sample_pkg/sample_pkg/calibdata.npz`) at import time, and that file is not installed by `sample_pkg`'s `setup.py`. `cam_node` will fail to start on any machine other than the one this path was written for.
- `myrt_description`'s URDF gives `base_link` a mass of roughly 37 kg, implausible for this chassis size, and the friction coefficients in `myrt_gazebo.xacro` were not adjusted to match. Both are exporter defaults and will make simulated dynamics behave differently from the real robot.
- The wheel controller's `wheel_separation` (0.17 m) and `wheel_radius` (0.033 m) match the TurtleBot3 Burger's dimensions rather than an independently measured value for this chassis; confirm these against the actual robot before trusting odometry.
- `myrt_description/urdf/ros2_control.xacro` declares `EffortJointInterface` transmission blocks that don't match the `velocity` command interfaces used by the joints. `gazebo_ros2_control` ignores the transmission blocks, so this is inert but confusing configuration.
- `myrt_description/rviz/display.rviz` references a `base_footprint` frame that no xacro file in this workspace defines; RViz will warn but continue.
- Package structure, dependencies, and the robot description have not been built or launched on ROS 2 Humble on the machine used to write this document. This includes the fixes made in this pass (removing the world-fixed joint, the missing `con_node` entry point, and the dependency corrections below), none of which have been build-tested.

## Notes on this pass

A few small fixes were made alongside this documentation update, all confirmed against the source before changing anything:

- `myrt_description`'s URDF previously fixed `base_link` to a `world` link with a `type="fixed"` joint. In Gazebo this pinned the chassis in place: the diff-drive controller could spin the wheels but the robot could not translate. The `world` link and the fixed joint have been removed so `base_link` is the free root link, which is standard for a simulated mobile robot.
- `sample_pkg`'s `setup.py` declared a `con_node` console script entry point pointing at `sample_pkg/con_pub.py`, a file that does not exist anywhere in the repository or its history. The entry point has been removed rather than reconstructed.
- `sample_pkg/package.xml` declared no runtime dependencies despite importing `rclpy`, `custom_msgs`, and `cv_bridge`; the missing dependencies have been added.
- `myrt_firmware/package.xml` declared `pyserial` as a dependency, which is not a resolvable rosdep key; the package already separately declared the correct key, `python3-serial`, so the invalid line was removed.
- `myrt_description` and `custom_msgs` were missing several build-time dependencies (`rclcpp`, `tf2`, `tf2_ros`, `sensor_msgs`, and others) that their `CMakeLists.txt` files require; these have been added to each package's `package.xml`.

## License

This project is licensed under the MIT License. `myrt_controller` and `myrt_description` still declare Apache 2.0 in their own `package.xml` files from an earlier pass and have been left as-is rather than silently overwritten; reconciling all five packages to one license is an open item.

The `ground_plane` and `sun` models under `myrt_description/models` are unmodified stock Gazebo models from the OSRF model database, originally authored by Nate Koenig, and keep their original attribution.
