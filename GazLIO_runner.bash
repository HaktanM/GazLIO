#!/bin/bash

# Ensure environment variables are set (you can also set these externally)
export DOCKER_DOCUMENTS=/home/haktanito/workspace/lidar_slam/folders
export DOCKER_DATASETS=/home/haktanito/workspace/lidar_slam/dataset
export DOCKER_FAST_LIO=/home/haktanito/workspace/lidar_slam/fast_lio
export DOCKER_LIVOX=/home/haktanito/workspace/lidar_slam/livox_driver2


# Run the rosbag_player container
xhost +local:docker
docker run -it \
    --mount type=bind,source="$DOCKER_DOCUMENTS",target=/documents \
    --mount type=bind,source="$DOCKER_DATASETS",target=/dataset \
    --mount type=bind,source="$DOCKER_FAST_LIO",target=/fast_lio \
    --mount type=bind,source="$DOCKER_LIVOX",target=/livox_driver2 \
    --privileged  --device /dev/input/js0:/dev/input/js0 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    humble bash -c "source /opt/ros/humble/setup.bash && \
                    cp /documents/robot.yaml ~/clearpath/robot.yaml &&
                    ros2 run clearpath_generator_common generate_bash -s ~/clearpath &&
                    source ~/clearpath/setup.bash && \
                    source /fast_lio/install/setup.bash && \
                    exec bash"
xhost -local:docker
