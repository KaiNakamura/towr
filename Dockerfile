FROM osrf/ros:noetic-desktop-full

WORKDIR /home

# Set up the workspace environment
ENV CATKIN_WS=catkin_ws
ENV ROS_PACKAGE_PATH=$CATKIN_WS/src/towr
RUN mkdir -p $ROS_PACKAGE_PATH

# Set environment variables
ENV SHELL=/bin/bash
ENV PATH=/usr/local/bin:$PATH

# Set the default command to bash
CMD ["/bin/bash"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-catkin-tools \
    ros-noetic-catkin \
    cmake \
    libeigen3-dev \
    coinor-libipopt-dev \
    libncurses5-dev \
    xterm \
    liboctomap-dev \
    ros-noetic-octomap-msgs \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-pybind11-catkin \
    libmpfr-dev \
    libgmp-dev
    # ros-noetic-xpp \
    # && apt-get clean && rm -rf /var/lib/apt/lists/*

# Clone Xpp
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/leggedrobotics/xpp.git

# Clone ifopt
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/ethz-adrl/ifopt.git

# Clone unitree_ros
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/unitreerobotics/unitree_ros.git

# Clone grid_map
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/ANYbotics/grid_map.git

# Clone elevation_mapping_cupy
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git

RUN /bin/bash -c "cd $CATKIN_WS && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin init && \
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    catkin build xpp go1_description ifopt && \
    source devel/setup.bash"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /root/.bashrc
RUN echo "source /home/$CATKIN_WS/devel/setup.bash" >> /root/.bashrc