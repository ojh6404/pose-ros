FROM nvidia/cuda:12.1.1-devel-ubuntu20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN rm /etc/apt/sources.list.d/cuda.list

# install essential packages
RUN apt update && apt install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    curl \
    wget \
    build-essential \
    git \
    lsb-release \
    software-properties-common \
    vim \
    tmux \
    cmake \
    zip \
    unzip \
    g++ \
    libc6-dev \
    libgtk2.0-dev \
    bzip2 \
    ca-certificates \
    checkinstall \
    gfortran \
    libjpeg8-dev \
    libtiff5-dev \
    pkg-config \
    yasm \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libdc1394-22-dev \
    libxine2-dev \
    libv4l-dev \
    qt5-default \
    libgtk2.0-dev \
    libtbb-dev \
    libatlas-base-dev \
    libfaac-dev \
    libmp3lame-dev \
    libtheora-dev \
    libvorbis-dev \
    libxvidcore-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    x264 v4l-utils \
    libprotobuf-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    libgflags-dev \
    libgphoto2-dev \
    libhdf5-dev \
    doxygen \
    libflann-dev \
    libboost-all-dev \
    proj-data \
    libproj-dev \
    libyaml-cpp-dev \
    cmake-curses-gui \
    libzmq3-dev \
    freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*


# install eigen 3.4.0 for FoundationPose
RUN cd / && wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz &&\
    tar xvzf ./eigen-3.4.0.tar.gz &&\
    cd eigen-3.4.0 &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make install

# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

# install ros core
RUN apt update && apt install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt update && apt install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-wstool \
    python3-vcstools \
    python-is-python3 \
    python3-pip \
    python3.9 \
    python3.9-dev \
    python3.9-venv \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt update && apt install -y --no-install-recommends \
    ros-noetic-image-transport-plugins \
    ros-noetic-jsk-tools \
    ros-noetic-jsk-common \
    ros-noetic-jsk-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# install point cloud library if needed
RUN apt update && apt install -y ros-noetic-jsk-pcl-ros ros-noetic-jsk-pcl-ros-utils &&\
    rm -rf /var/lib/apt/lists/*

# install pybind11 for FoundationPose
RUN cd / && git clone https://github.com/pybind/pybind11 &&\
    cd pybind11 && git checkout v2.10.0 &&\
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_INSTALL=ON -DPYBIND11_TEST=OFF &&\
    make -j6 && make install

SHELL ["/bin/bash", "-c"]


# trick for cuda because cuda is not available when building docker image
ENV FORCE_CUDA="1" TORCH_CUDA_ARCH_LIST="6.1;7.0;7.5;8.0;8.6;8.9+PTX"

########################################
########### WORKSPACE BUILD ############
########################################
# Installing catkin package
RUN rosdep init && rosdep update && apt update
RUN mkdir -p ~/catkin_ws/src
RUN git clone https://github.com/ojh6404/pose-ros.git ~/catkin_ws/src/pose-ros
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/catkin_ws/src/pose-ros && \
    rosdep install --from-paths . -i -r -y
RUN cd ~/catkin_ws/src/pose-ros && ./prepare.sh
RUN cd ~/catkin_ws && catkin init && catkin build &&\
    rm -rf ~/.cache/pip

# # to avoid conflcit when mounting
RUN rm -rf ~/catkin_ws/src/pose-ros/launch
RUN rm -rf ~/catkin_ws/src/pose-ros/node_scripts

# ########################################
# ########### ENV VARIABLE STUFF #########
# ########################################
RUN touch ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# CMD ["bash"]
