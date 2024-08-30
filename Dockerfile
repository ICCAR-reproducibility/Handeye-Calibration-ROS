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

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main"

RUN apt-get update \
 && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg \
 && rm -rf /var/lib/apt/lists/*

# RUN cd /catkin_ws \
#  && source /opt/ros/${ROS_DISTRO}/setup.bash \
#  && apt-get update \
#  && rosdep install -y --from-paths src --ignore-src \
#  && rm -rf /var/lib/apt/lists/*

# RUN cd /catkin_ws \
#   && source /opt/ros/${ROS_DISTRO}/setup.bash \
#   && catkin_make