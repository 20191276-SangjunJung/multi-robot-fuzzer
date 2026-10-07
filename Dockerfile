FROM osrf/ros:humble-simulation

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash"]