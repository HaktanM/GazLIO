#!/bin/bash
# Run the rosbag_player container
xhost +local:docker
docker run -it \
    --privileged  --device /dev/input/js0:/dev/input/js0 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    gaz_lio bash -c " source /opt/ros/jazzy/setup.bash && \
                    ros2 run clearpath_generator_common generate_bash -s ~/clearpath &&
                    source ~/clearpath/setup.bash && \
                    source ~/fast_lio/install/setup.bash && \
                    terminator"
xhost -local:docker