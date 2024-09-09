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

RUN apt-get update \
 && apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config \
                       libavcodec-dev libavformat-dev libswscale-dev \
                       python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev \
                       libpng-dev libtiff-dev libdc1394-22-dev \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main" \
 && apt-get update \
 && apt-get install -y libjasper1 libjasper-dev \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir opencv \
 && curl -L "https://github.com/opencv/opencv/archive/refs/tags/3.4.0.tar.gz" | tar xz --strip=1 -C "opencv"

ADD https://github.com/opencv/opencv_contrib/archive/refs/tags/3.4.0.tar.gz opencv_contrib.tar.gz

RUN tar zxf opencv_contrib.tar.gz

RUN cd opencv \
 && mkdir build \
 && cd build \
 && cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib-3.4.0/modules/ \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDA=OFF \
    -D WITH_CUBLAS=OFF \
    -D WITH_CUFFT=OFF \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D CMAKE_INSTALL_PREFIX=/usr/local .. \
 && make -j5 install

ADD https://github.com/google/glog/archive/refs/tags/v0.4.0.tar.gz glog.tar.gz

RUN tar zxf glog.tar.gz

RUN cd glog-0.4.0 \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make install

RUN git clone https://github.com/ros-perception/vision_opencv.git \
  && cd vision_opencv \
  && git checkout melodic \
  && cd cv_bridge \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && mkdir build && cd build \
  && cmake .. \
  &&  make -j5 install

RUN cd /opt/ros/melodic \
 && rm -rf lib/libcv_bridge.so \
 && rm -rf include/cv_bridge \
 && rm -rf share/cv_bridge

RUN cd /catkin_ws \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep install -y --from-paths src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

RUN cd /catkin_ws \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && catkin_make
