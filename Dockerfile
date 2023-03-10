FROM ros:humble

ADD . /deplex-ros/src

WORKDIR /deplex-ros/src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
     colcon build \
    --packages-select \
    deplex_ros

RUN sed --in-place --expression \
    '$isource "./install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["ros2", "launch", "deplex_ros", "deplex_ros_launch.py"]