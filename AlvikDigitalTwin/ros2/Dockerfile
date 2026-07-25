FROM ros:humble
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install websockets
WORKDIR /ros2_ws
COPY src/ src/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"
RUN ln -sf /ros2_ws/src/alvik_bridge/alvik_bridge/alvik_ros_bridge.py \
           /ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_ros_bridge.py && \
    ln -sf /ros2_ws/src/alvik_bridge/alvik_bridge/alvik_pid_controller.py \
           /ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_pid_controller.py
RUN mkdir -p /ros2_ws/install/alvik_bridge/lib/alvik_bridge && \
    ln -s /ros2_ws/install/alvik_bridge/bin/alvik_ros_bridge \
          /ros2_ws/install/alvik_bridge/lib/alvik_bridge/alvik_ros_bridge && \
    ln -s /ros2_ws/install/alvik_bridge/bin/alvik_pid_controller \
          /ros2_ws/install/alvik_bridge/lib/alvik_bridge/alvik_pid_controller
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export PATH=/ros2_ws/install/alvik_bridge/bin:\$PATH" >> ~/.bashrc
