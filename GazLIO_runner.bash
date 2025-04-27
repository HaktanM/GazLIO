#!/bin/bash

# Run the rosbag_player container
xhost +local:docker
docker run -it \
    --privileged  --device /dev/input/js0:/dev/input/js0 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    gaz_lio bash -c "source /opt/ros/humble/setup.bash && \
                            cp ~/fast_lio/src/fast_lio/robot_w200.yaml ~/clearpath/robot.yaml &&
                            ros2 run clearpath_generator_common generate_bash -s ~/clearpath &&
                            source ~/clearpath/setup.bash && \
                            source ~/fast_lio/install/setup.bash && \
                            sed -i 's|<real_time_factor>[^<]*</real_time_factor>|<real_time_factor>0.5</real_time_factor>|' /opt/ros/humble/share/clearpath_gz/worlds/warehouse.sdf && \
                            terminator"
xhost -local:docker