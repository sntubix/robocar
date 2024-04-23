FROM docker.io/ros:humble

# install tools and libraries
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    sudo vim tmux clang gcc gdb cmake git curl unzip clinfo ffmpeg python3-pip \
    python3-matplotlib rapidjson-dev libeigen3-dev libgps-dev libopencv-dev \
    libsdl2-dev libserial-dev opencl-headers ocl-icd-opencl-dev intel-opencl-icd
RUN update-alternatives --set c++ /usr/bin/clang++

# install ROS components
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-common-interfaces ros-humble-rviz2 ros-humble-pcl-ros \
    ros-humble-cv-bridge ros-humble-image-transport-plugins

# install YOLOv8
RUN pip install ultralytics

# clear tmp files and cache
RUN rm -rf /tmp/libs && rm -rf /var/lib/apt/lists/*

# create robocar user
RUN useradd -s /bin/bash -m robocar

USER robocar
WORKDIR /host
ENTRYPOINT /bin/bash
