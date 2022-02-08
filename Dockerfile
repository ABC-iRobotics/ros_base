FROM ros:noetic-robot

# install build tools
RUN apt-get update && apt-get install -y \
      git-all \
      python3-catkin-tools \
      ros-noetic-vision-msgs \
      ros-noetic-cv-bridge \
      ros-noetic-vision-opencv \
      ros-noetic-moveit \
      ros-noetic-franka-description \
      ros-noetic-ros-control \
      ros-noetic-ros-controllers && \
    rm -rf /var/lib/apt/lists/*


# clone ros packages
ENV ROS_UNDERLAY_WS /usr/underlay_ws
RUN mkdir -p $ROS_UNDERLAY_WS/src

WORKDIR $ROS_UNDERLAY_WS

RUN git -C src clone \
      https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

RUN git -C src clone \
      -b calibration_devel \
      https://github.com/fmauch/universal_robot.git

# install ros package dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y \
      --from-paths \
        src \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# build underlay workspace
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_UNDERLAY_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

ENV ROS_WS /usr/catkin_ws
RUN mkdir -p $ROS_WS/src

WORKDIR $ROS_WS

CMD ["bash"]