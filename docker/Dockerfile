FROM docker.io/ros:humble

# install tools and libraries
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    vim git ffmpeg clang gcc gdb cmake python3-pip python3-matplotlib \
    rapidjson-dev libeigen3-dev libopencv-dev libsdl2-dev libserial-dev \
    opencl-headers ocl-icd-opencl-dev intel-opencl-icd libzmq3-dev
RUN update-alternatives --set c++ /usr/bin/clang++

# install ROS components
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    ros-humble-common-interfaces ros-humble-rviz2 ros-humble-pcl-ros \
    ros-humble-cv-bridge ros-humble-image-transport-plugins

# install python libraries
RUN pip install ultralytics pygame pyzmq vidgear ffpyplayer matplotlib

# clear tmp files and cache
RUN rm -rf /tmp/libs && rm -rf /var/lib/apt/lists/*

# create robocar user
RUN useradd -s /bin/bash -m robocar

USER robocar
WORKDIR /ws
ENTRYPOINT ["bash"]
