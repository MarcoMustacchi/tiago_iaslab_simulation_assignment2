<p align="center">
  <img src="https://github.com/MarcoMustacchi/MarcoMustacchi.github.io/blob/main/assets/img/icons/UniPD_logo.svg" width="150">
</p>

<h1 align="center">Intelligent Robotics - Final Project <br> UniPd</h1>

<a href="https://github.com/MarcoMustacchi/IntelligentRoboticsProject/raw/master/Assignment-2.pdf">📄Task2</a>
<a href="https://github.com/MarcoMustacchi/IntelligentRoboticsProject/raw/master/IR_Assignment2.pdf">📄Report2</a>

## Simple guide to start a Docker container from a Docker image

### Docker container without GUI
Docker expects to have a single Dockerfile (called exactly Dockerfile) in the current folder. Then, starting from that, using:
```bash
docker build -t ros-tiago-project-2 .
```
you create a Docker image called `ros-tiago-project-2` in the current folder. Then using:
```bash
docker run -it ros-tiago-project-2
```
you start a Docker container from the `ros-tiago-project-2` image.

### Docker container with GUI
Docker expects to have a single Dockerfile (called exactly Dockerfile) in the current folder. Then, starting from that, using:
```bash
docker build -t ros-tiago-project-2 .
```
you create a Docker image called `ros-tiago-project-2` in the current folder. <br>
Allow local docker containers to access the X server
```bash
xhost +local:docker
```
Then using:
```bash
docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros-tiago-project-2
```
you start a Docker container from the `ros-tiago-project-2` image. <br>
Inside the Docker container, you can use `terminator` to easily open multiple terminals
```bash
terminator
```
> **Note**: using `terminator` you start in the /root directory. You can simply type
> ```bash
> cd ..
> ```
> to go in the parent folder, which is the default starting folder of Docker.

After you’re done testing, reset the permissions:
```bash
xhost -local:docker
```
> **Note**: it seems like the GUI applications are generally working (e.g., terminator and rqt_image_view), <br>
> but still experiencing issues specifically with RViz and Gazebo. <br>
> These applications rely on 3D rendering and OpenGL, which can be tricky to get working correctly in a Docker container.

# Installation Guide
#### 0: Source ROS installation using `~/.bashrc`
```bash
echo "source /opt/ros/noetic/setup.bash">> ~/.bashrc
```
#### 1: Create Workspace
```bash
mkdir -p ~/tiago_public_ws/src
```
#### 2: Navigate to Workspace
```bash
cd ~/tiago_public_ws/src
```
#### 3: Initialize the Workspace
```bash
catkin_init_workspace 
```
#### 4: Navigate to Workspace
```bash
cd ~/tiago_public_ws
```
#### 5: Build the Workspace
```bash
catkin build
```
This will create the necessary build, devel, and src directories. 
It also configures the workspace for ROS development.

# Dependencies
**NB.** This guide has been extracted from the official Wiki available at Robots/TIAGo/Tutorials so please, if you have any problem during the following steps, please take a look also at the official documentation.

TIAGO is a service robot produced by PAL Robotics. It is essentially a humanoid robot with the kinematic model of a simple mobile manipulator (differential mobile base + 6dof anthropomorphic manipulator). At the end of this install procedure you will be able to reproduce a very complex simulation that replicates all the features and capability of the real TIAGO robot. Below the results obtained at the end of this install guide.

#### 1. Navigate to workspace
```bash
cd ~/tiago_public_ws
```

#### 2. Install the TIAGO packages (inside your workspace folder)
```bash
wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall
```
```bash
rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall
```

> ### Warning: version compatibility
> When you download the required repositories using `rosinstall`, the repositories will be cloned from the latest version by default. <br>
> To get a specific tag, you will need to do this manually after the initial clone.
> ##### Manually Checkout a Specific Tag:
> After cloning the repositories, navigate to the tiago_navigation directory and check out the specific tag you need.
> ```bash
> cd ~/tiago_public_ws/src/tiago_navigation
> ```
> ```bash
> git checkout 2.1.5
> ```

#### 3. Run this command to make sure that all dependencies are installed
```bash
rosdep update
```

#### 4. Run the following to make sure that all dependencies referenced in the workspace are installed
```bash
rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"
```

#### 5. Build the workspace
```bash
catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)
```

#### 6. Source the ROS workspace using `~/.bashrc`
```bash
echo "source ~/tiago_public_ws/devel/setup.bash">> ~/.bashrc
```

Now you can close and reopen your shell. 
From this moment on, your shell will be always updated and pointing to your ROS workspace.

#### 7. Test your simulation
To launch the simulation of the TIAGo **Steel**, execute:
```bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel
```

The **Titanium** version can be launched as follows:
```bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium
```

#### 8. Delete all the installation scripts and setup files
```bash
cd ~/tiago_public_ws && rm tiago_public-noetic.rosinstall
```
and
```bash
cd ~/tiago_public_ws/src && rm .rosinstall && rm setup.bash setup.sh setup.zsh
```

## Additional dependency for Assignment 2

You need to add the package `gazebo_ros_link_attacher` in your workspace

```bash
cd ~/tiago_public_ws/src
```
clone the GitHub repository
```bash
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
```
build the package
```bash
catkin build gazebo_ros_link_attacher
```

# Package Installation
#### Clone the Repository package 
```bash
git clone https://github.com/MarcoMustacchi/tiago_iaslab_simulation_assignment2.git
```
#### Rename the package
```bash
mv tiago_iaslab_simulation_assignment2 tiago_iaslab_simulation
```
#### Build the package
```bash
catkin build tiago_iaslab_simulation
```

# Fix a possible issue with LaserScan in simulation
The LaserScan in the simulation may not be visible.  
The problem is related to the driver version of the graphic card that you used.

#### To solve the problem, try the following instructions:
* Go to **Software & Updates -> Additional driver** (see picture below)  
  <img src="https://github.com/MarcoMustacchi/IntelligentRoboticsProject/blob/master/Fix_Laser_Scan_Issue.jpeg">
* If you are using the native Ubuntu driver (X.org Nouveau), switch to the NVIDIA driver.
* Try using driver version 470.
* If version 470 does not work, try other versions. The driver version may vary depending on your NVIDIA model.

## Usage

To build the package see [build](#markdown-header-build_1).

To use the package you have to

- start the gazebo simulation:
  ```bash
  roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
  ```
- start the apriltag stack:
  ```bash
  roslaunch tiago_iaslab_simulation apriltag.launch
- start the navigation stack:
  ```bash
  roslaunch tiago_iaslab_simulation navigation.launch
  ```
-  wait until TIAGO has tucked its arm
-  start the package:
  ```bash
  roslaunch tiago_iaslab_simulation pick_place.launch  [corridor:=true]
  ```

## Build

You can use `catkin`:
```bash
catkin build tiago_iaslab_simulation
```
