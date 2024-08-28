# syntax=docker/dockerfile:1.4

# Use the official ROS Noetic base image
FROM ros:noetic-ros-core-focal AS base

# Set environment variables for ROS
ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND noninteractive

# Stage 1: Install dependencies that rarely change
FROM base AS dependencies

# Update and install necessary dependencies
RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
    apt-utils \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    git \
    python3-pip \
    python3-rosdep

# Initialize rosdep
RUN rosdep init && rosdep update && apt update

# Install required ROS packages
RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-hector-gazebo-worlds \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator

# Stage 2: Clone the repository and install ROS dependencies
FROM dependencies AS build

# Set up the workspace
RUN mkdir -p /home/src
WORKDIR /home/src

# Clone the POLARIS_GEM_e2 repository
RUN --mount=type=cache,target=/root/.cache \
    git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Install dependencies using rosdep
WORKDIR /home
RUN --mount=type=cache,target=/root/.cache \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Stage 3: Build the workspace
FROM build AS final

# Build the workspace
RUN --mount=type=cache,target=/root/.cache \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the setup file
#RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/devel/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /home/devel/setup.bash && bash"]

# Clean up apt
RUN rm -rf /var/lib/apt/lists/*

# Default command
CMD ["bash"]