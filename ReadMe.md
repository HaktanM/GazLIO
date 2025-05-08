# What is This Repository?

This repository provides a [Dockerfile](Dockerfile_GazLIO) that installs [Gazebo](https://gazebosim.org/home) and a slightly modified version of [FastLIO](https://github.com/hku-mars/FAST_LIO), a state-of-the-art Lidar-Inertial Odometry (LIO) system. The simulation environment in Gazebo includes a ground vehicle equipped with a LiDAR, IMU, and a stereo camera pair.

You can control the vehicle using a joystick, while FastLIO simultaneously processes IMU and LiDAR measurements and visualizes the estimated trajectory with map in real-time.

<div align="center">
  <img src="Visualization/GazLIO.gif" alt="Visualization of the Overall System" width="80%">
</div>

You can watch the [full video](https://youtu.be/AwSmk49Mt2I)

# Building the Docker Image

First, build the docker file. This will dowload and install all the necessary libraries and tools to a docker image including Gazebo simulator and Fast LIO.
```bash
cd GazLIO
docker build -t gaz_lio -f Dockerfile_GazLIO .
```

# Running The System

The overall system is composed of 3 subsystems
- Simulator: Clearpath
- Controller: Joystick
- State Estimator: FastLIO


### Terminator
Each subsytem should be able to communicate with each other. **Terminator** is a useful terminal which enables us to connect each subsytem through ROS.

### Start the docker image
First initialize the docker image
```bash
cd GazLIO
bash run_GazLIO.bash
```

This will automatically start the **terminator**.

- **Left-click on the Terminator window**. Then press <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>E</kbd> or <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>O</kbd> to split the terminal horizontally or vertically. Each split terminal can be used to interact with ROS. **Create as many terminal panels as needed.**
- You can change the real time factor of the simulation from `GazLIO_runner.bash`
    ```
    sed -i 's|<real_time_factor>[^<]*</real_time_factor>|<real_time_factor>0.2</real_time_factor>|' /opt/ros/jazzy/share/clearpath_gz/worlds/warehouse.sdf
    ```

### All Steps at a Glance
Follow these steps to run the full system. For detailed instructions, refer to the upcoming sections.
```bash
# In one of the terminator terminals, start the simulator
ros2 launch clearpath_gz simulation.launch.py world:=warehouse

# In one of the terminator terminals, start the joystick driver
ros2 run joy joy_node

# In one of the terminator terminals, start the joystick interpreter
cd /root/joystick
source install/setup.bash
ros2 run joy2twist joy2twist --ros-args --remap /cmd_vel:=/ground_vehicle/platform/cmd_vel

# Once the simulator is ready, start FastLIO in one of the terminator terminals.
ros2 launch fast_lio mapping.launch.py config_file:=gazebo.yaml
```
### Step by Step Explanations

#### Set Simulation Real Time Factor for Efficient Data Collection
It is recommended to decrease the simulation time rate. Without adjusting the time rate, data will continue to accumulate at a fast rate, potentially causing issues with saving or processing the data effectively.

To adjust the simulation time flow, follow these steps:
1.  Navigate to the world SDF file:
    ```bash
    cd /opt/ros/jazzy/share/clearpath_gz/worlds/
    nano warehouse.sdf
    ```
2. Modify the **real_time_factor**
    ```xml
    <sdf version='1.7'>
    <world name='warehouse'>
        <physics type="ode">
        <max_step_size>0.003</max_step_size>
        <real_time_factor>0.2</real_time_factor>
    ```
3. Save and close. Then, restart the simulator to apply the changes.

#### Start Simulator
To launch the simulator, copy and paste the following script into one of the terminator terminals:
```bash
ros2 launch clearpath_gz simulation.launch.py world:=warehouse
```
**Note:** Currently, you can try other worlds as well.
- warehouse
- office
- pipeline
- orchard

#### Start Joystick Control
Joystick control enable us to drive the vehicle from a joystick. In another terminator terminal, run the following command to start the joystick driver:
```bash
ros2 run joy joy_node
```

Next, we need to convert the joystick outputs into a format that Gazebo expects. This is done using a dedicated package.

Finally, run the following commands:

```bash
cd /root/joystick
source install/setup.bash
ros2 run joy2twist joy2twist --ros-args --remap /cmd_vel:=/cmd_vel:=/ground_vehicle/platform/cmd_vel
```

Once this is done, you can use the joystick to drive the vehicle in the simulator.

The button mapping shown below is adapted from the [joy2twist Github page](https://github.com/husarion/joy2twist.git). For more information and detailed documentation, please refer to the original [joy2twist Github page](https://github.com/husarion/joy2twist.git).
<div align="center">
  <img src="https://raw.githubusercontent.com/husarion/joy2twist/ros2/.docs/gamepad-legend-panther.png" alt="JoyStick Controller" width="75%">
</div>

**Warning:** If the vehicle does not respond to joystick inputs, try closing and reopening the simulator. To further troubleshoot motion control, check the `/ground_vehicle/platform/cmd_vel` topic to verify whether the joystick is publishing velocity commands.
```bash
ros2 topic echo /ground_vehicle/platform/cmd_vel
```


#### Start Fast LIO
Before starting the FastLIO, wait untill the simulator is ready. Once the simulator is ready, use the following command:
```bash
ros2 launch fast_lio mapping.launch.py config_file:=gazebo.yaml
```

# Data Collection for Offline Examiantion
You can collect data for offline examination. For example, you may wish to record the following data streams for later analysis:
- IMU data
- Lidar Data
- Left and right camera frames

To start recording the data, use the following command:
```bash
ros2 bag record /ground_vehicle/sensors/imu_0/data /ground_vehicle/sensors/lidar3d_0/points /ground_vehicle/sensors/camera_0/color/image /ground_vehicle/sensors/camera_1/color/image -o _warehouse
```
This will create a ros2 bag file. 

Unfortunately, the ground truth pose is not available through ROS topics, but it is published by Gazebo. You can record the ground truth pose directly from Gazebo.
```bash
gz topic -e -t /model/ground_vehicle/robot/pose > groundtruth.txt
```

This creates a file with groundtruth. You may want to convert the grountruth into a TUM format for evaluation. You can use [PrepareGT.py](https://github.com/HaktanM/GazLIO/blob/main/PrepareGT.py) to convert the groundtruth format. 

In order to see other gazebo topics, use this commend:
```bash
gz topic -l
```


# Additional Notes
## Testing FastLIO with Ros2 Bag

### HKU Dataset
Dowload the HKU ros2 bag from [google drive](https://drive.google.com/drive/folders/16IUNQagundFwNg3uJFNCSdyLr9VdAxVp?usp=sharing)

1. First initialize the docker image
    ```bash
    bash run_GazLIO.bash
    ```
2. Split the terminator terminal into 2 (<kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>E</kbd>).

3. In one terminator window, start FastLIO
    ```
    ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
    ```

4. Run the ros2 bag from another terminal
    ```bash
    cd /PATH/TO/datasets
    ros2 bag run HKU_MB
    ```


### Clearpath Simulated Data
Dowload the simulation ros2 bags from [google drive](https://drive.google.com/drive/folders/16IUNQagundFwNg3uJFNCSdyLr9VdAxVp?usp=sharing)

1. First initialize the docker image
    ```bash
    bash run_GazLIO.bash
    ```

2. Split the terminator terminal into 2 (<kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>E</kbd>).

3. In one terminator window, start FastLIO
    ```bash
    ros2 launch fast_lio mapping.launch.py config_file:=gazebo.yaml
    ```

4. Run the ros2 bag from another terminal
    ```bash
    cd /PATH/TO/datasets
    ros2 bag run warehouse
    ```
