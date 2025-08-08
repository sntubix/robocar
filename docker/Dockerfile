FROM docker.io/ros:jazzy

# install tools and libraries
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    vim git ffmpeg clang gdb cmake python3-pip python3-venv rapidjson-dev \
    libzmq3-dev libeigen3-dev libopencv-dev libomp-dev libpcap-dev libsdl2-dev \
    opencl-headers ocl-icd-opencl-dev intel-opencl-icd libgstreamer1.0-dev \
    libgstrtspserver-1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
RUN update-alternatives --set c++ /usr/bin/clang++

# install ROS components
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    ros-jazzy-common-interfaces ros-jazzy-rviz2 ros-jazzy-pcl-ros \
    ros-jazzy-cv-bridge ros-jazzy-image-transport-plugins

# install python libraries
RUN python3 -m venv /venv
RUN /venv/bin/pip install ultralytics pygame pyzmq vidgear
RUN /venv/bin/pip install catkin_pkg empy lark
ENV PATH="/venv/bin:$PATH"

# clear cache
RUN rm -rf /var/lib/apt/lists/*

USER ubuntu
WORKDIR /ws
ENTRYPOINT ["bash"]
