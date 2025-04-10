FROM osrf/ros:noetic-desktop-full

WORKDIR /home

ARG HSL57_PATH=null
ARG HSL97_PATH=null

# Set up the workspace environment
ENV CATKIN_WS=catkin_ws
ENV ROS_PACKAGE_PATH=$CATKIN_WS/src/towr
RUN mkdir -p $ROS_PACKAGE_PATH
RUN mkdir repos

# Set environment variables
ENV SHELL=/bin/bash
ENV PATH=/usr/local/bin:$PATH

# Set the default command to bash
CMD ["/bin/bash"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    cmake \
    python3-catkin-tools \
    ros-noetic-catkin \
    libeigen3-dev \
    coinor-libipopt-dev \
    libncurses5-dev \
    xterm \
    liboctomap-dev \
    ros-noetic-octomap-msgs \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-pybind11-catkin \
    libmetis-dev \
    libmpfr-dev \
    gfortran \
    libgmp-dev
# && apt-get clean && rm -rf /var/lib/apt/lists/*

# Clone Xpp
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/leggedrobotics/xpp.git

# Clone unitree_ros
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/unitreerobotics/unitree_ros.git

# Clone grid_map
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/ANYbotics/grid_map.git

# Clone elevation_mapping_cupy
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git

# Install cpptrace
RUN cd repos && \
    git clone https://github.com/jeremy-rifkin/cpptrace.git && \
    cd cpptrace && \
    git checkout v0.7.5 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j && \
    sudo make install

# Snopt
ARG SNOPT_PATH=null
ADD ${SNOPT_PATH} /home/snopt
ENV SNOPT_DIR=/home/snopt

ARG SNOPT_LICENSE=null
ADD ${SNOPT_LICENSE} /home/licenses/snopt.lic
ENV SNOPT_LICENSE=/home/licenses/snopt.lic

# Change Snopt directory structure to work with Ifopt
RUN mkdir -p /home/snopt/include /home/snopt/lib && \
    mv /home/snopt/snopt.h /home/snopt/snoptProblem.hpp /home/snopt/snopt_cwrap.h /home/snopt/sqopt.h /home/snopt/sqoptProblem.hpp /home/snopt/sqopt_cwrap.h /home/snopt/include && \
    mv /home/snopt/libsnopt7_cpp.a /home/snopt/libsnopt7_cpp.so /home/snopt/lib && \
    ln -s /home/snopt/lib/libsnopt7_cpp.so /home/snopt/lib/libsnopt7.so

ADD ${HSL57_PATH} /home/repos/hsl57/
RUN cd repos/hsl57 && \
    ./configure && \
    make -j1 && \   
    sudo make install && \
    ln -sf /usr/local/lib/libhsl_ma57.so /opt/ros/noetic/lib/libhsl.so

ADD ${HSL97_PATH} /home/repos/hsl97/
RUN cd repos/hsl97 && \
    ./configure FC="gfortran -fopenmp" CC="gcc -fopenmp" CXX="g++ -fopenmp" LDFLAGS="-lgomp"&& \
    make -j1 && \   
    sudo make install && \
    ln -sf /usr/local/lib/libhsl_ma97.so /opt/ros/noetic/lib/libhsl.so

# Ifopt
RUN cd repos && \
    git clone https://github.com/KaiNakamura/ifopt.git && \
    cd ifopt && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_SNOPT=ON -DCMAKE_BUILD_TYPE=Release && \
    make -j && \
    sudo make install && \
    ln -sf /usr/local/lib/libifopt_core.so /opt/ros/noetic/lib/libifopt_core.so

# Add the directory containing libifopt_core.so to the LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

RUN /bin/bash -c "cd $CATKIN_WS && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin init && \
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    catkin build xpp go1_description convex_plane_decomposition_ros && \
    source devel/setup.bash"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /root/.bashrc
RUN echo "source /home/$CATKIN_WS/devel/setup.bash" >> /root/.bashrc