# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update package index
RUN apt-get update 

# Install essential tools
RUN apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    build-essential \
    python3

# Clean up package lists
RUN rm -rf /var/lib/apt/lists/*

# Add the ROS Noetic package sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Update package index again after adding ROS repository
RUN apt-get update

# Install ROS Noetic desktop full (includes RViz and Gazebo)
RUN apt-get install -y ros-noetic-desktop-full

# Install ROS tools
RUN apt-get install -y \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool

# Make sure that all ROS dependencies are installed
RUN rosdep init && rosdep update

# Create and Initialize ROS workspace
RUN mkdir -p ./catkin_ws/src \
    && cd ./catkin_ws/src \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"
    
# Build ROS workspace
RUN cd ./catkin_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
    
# Get and Install the ROS TiaGo packages
RUN cd ./catkin_ws \
    && wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall \
    && rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall
    
# Manually checkout a Specific Tag
RUN cd ./catkin_ws/src/tiago_navigation \
    && git checkout 2.1.5
    
# Make sure that all ROS dependencies referenced in the workspace are installed  
RUN cd ./catkin_ws \
    && rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver" 

# Build the workspace 
RUN cd ./catkin_ws/src \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)"
    
# Delete all the installation scripts and setup files
RUN cd /catkin_ws && rm tiago_public-noetic.rosinstall \
    && cd /catkin_ws/src && rm .rosinstall \
    && cd /catkin_ws/src && rm setup.bash setup.sh setup.zsh
    
# Install and Build ROS gazebo_ros_link_attacher package
RUN cd ./catkin_ws/src \
    && git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build gazebo_ros_link_attacher" 
    
# Install and Build ROS tiago_iaslab_simulation package for Assignment 2
RUN cd ./catkin_ws/src \
    && git clone https://github.com/MarcoMustacchi/tiago_iaslab_simulation_assignment2.git \
    && mv tiago_iaslab_simulation_assignment2 tiago_iaslab_simulation \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build tiago_iaslab_simulation"  
    
# Source the ROS setup.bash (for ROS and for package) for future terminal sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
    
# Create a symbolic link for python to python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install Terminator
RUN apt-get update && apt-get install -y terminator \
    && rm -rf /var/lib/apt/lists/*
     
# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["bash"]
