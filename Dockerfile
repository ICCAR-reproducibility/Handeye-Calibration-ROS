FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y curl apt-transport-https \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src \
 && cd /catkin_ws/src \
 && git clone --recursive https://github.com/ICCAR-reproducibility/Handeye-Calibration-ROS.git

RUN mkdir -p /etc/apt/keyrings \
 && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update \
 && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg \
 && rm -rf /var/lib/apt/lists/*

RUN curl -O http://ceres-solver.org/ceres-solver-1.14.0.tar.gz \
 && tar zxf ceres-solver-1.14.0.tar.gz

RUN apt-get update \
 && apt-get install -y cmake libgoogle-glog-dev libgflags-dev \
                       libatlas-base-dev libeigen3-dev libsuitesparse-dev \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir ceres-bin \
 && cd ceres-bin \
 && cmake ../ceres-solver-1.14.0 \
 && make -j3 install

RUN git clone https://github.com/strasdat/Sophus.git \
 && cd Sophus/ \
 && git checkout v1.0.0 \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make install

# RUN cd /catkin_ws \
#  && source /opt/ros/${ROS_DISTRO}/setup.bash \
#  && apt-get update \
#  && rosdep install -y --from-paths src --ignore-src \
#  && rm -rf /var/lib/apt/lists/*

# RUN cd /catkin_ws \
#   && source /opt/ros/${ROS_DISTRO}/setup.bash \
#   && catkin_make