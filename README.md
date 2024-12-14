## Docker
Very briefly, Docker is a tool that provides a simple and efficient way to pack everything needed for a specific application in one container. You can also look at it as a lightweight virtual machine running on your computer.

Basic information about Docker and its main concepts can be found [here](https://github.com/larics/docker_files/wiki). Of course, you can also take a look at the [official website](https://www.docker.com/). Don't follow any instructions from these links just yet. They are provided as a general overview and reference you can use in the future. Detailed step-by-step instructions are given below.

#### Prerequisites
You must have Ubuntu OS installed on your computer. Ideally, this would be Ubuntu 24.04, but another version should work as well. 

#### Step-by-step instructions
Follow these [instructions](https://docs.docker.com/engine/install/ubuntu/) to install the Docker engine.

Then follow these [optional steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non-root user. If you skip this, every `docker` command will have to be executed with `sudo`. Skip the _"Note: To run Docker without root privileges, see Run the Docker daemon as a non-root user (Rootless mode)."_ part. This is just a note and we do not need it.

Docker containers are intended to run inside your terminal. In other words, you won't see a desktop like in regular virtual machines. However, graphical applications can still run in the container if you give them permission. To do that, execute
```bash
xhost +local:docker
```
To avoid having to do this every time, we can add that command to our `.profile` file which executes on every login.
```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

If you have an NVIDIA GPU, please install `nvidia-container-toolkit` by following [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).


## Run Crazyflies in the Docker container

### Setting up

Clone the [repository](https://github.com/marijana23/icuas_competition_2025.git):
```
git clone https://github.com/marijana23/icuas_competition_2025.git
```
Add  to  `~/.bashrc` and source it, or type in the current terminal:
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
# To install ros1_bridge and ROS Noetic set the argument INSTALL_BRIDGE to true.
# Otherwise set it to false, and it will only install ROS2.
docker build --ssh default -t crazysim_icuas_img .

# Run the crazysim_img2 container for the fist time
./first_run.sh

# This will create docker container crazyswarm_container and position you into the container

# Start the crazysim_cont:
docker start -i crazysim_icuas_cont

# Start the crazysim_cont in another terminal, while it is already started:
docker exec -it crazysim_icuas_cont bash

# Stop the conatainer
docker stop crazysim_icuas_cont

# Delete the container
docker rm crazysim_icuas_cont

```
The docker contains packages for crazyflies simulator [CrazySim](https://github.com/gtfactslab/CrazySim). General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

> [!NOTE]
> The ros2 workspace is located in /root/CrazySim/ros2_ws

Generaly, since here are installed ROS1 and ROS2, sourcing should be considered and before running any ros2 package you need to source ros2  using alias:

```
ros2_ws
source_ros2
```
Notice that in this case aliases are already used in .bashrc and it is not needed to do that, however if you intend to use ros1_bridge, you will need to do it. 

### RUN ICUAS EXAMPLE

Navigate to `/root/CrazySim/ros2_ws/relay_race/startup`. Start the example: 

```
./start.sh
```

If needed, do `chmod +x start.sh` before. It starts the example with 4 crazyflies and 5 aruco markers. In the first pane of the second window you can start teleop_twist, which is already in history. It controls cf_1.

#### Interesting topics

* `cf_x/odom` - odometry (can be used for feedback)
* `cf_x/image` - image from camera (the name of the topic can be changed)
* `cf_x/battery` - percentage of the battery and general state

#### How can crazyflies be controlled:
* `cf_x/cmd_attitude` - low level controller (it is used in the MPC scripts)
* `cf_x/cmd_hover` - velocities for horizontal hovering, height can be changed by dynamically changing hovering height 
* `cf_x/start_trajectory`,`cf_x/upload_trajectory` - services for executing trajectories
* `cf_x/go_to` - service to define point to which cf should go


### Running MPC experiments

In the folder `/root/startup` is `start.sh` script that starts the session with the examples from [CraySim page](https://github.com/gtfactslab/CrazySim).

Before starting the session, please comment out all crazyflies that are not in Gazebo in file `ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml` ([Check this](https://github.com/gtfactslab/CrazySim?tab=readme-ov-file#configuration) for more info).

When starting the example script, if the frames of crazyflies do not appear in rviz and you wait too long, please check [this issue](https://github.com/gtfactslab/CrazySim/issues/1#issuecomment-1933212957).

### ROS1 info
This package is primarily supported in ROS2, and we strongly suggest using ROS2 for development. If you insist on using ROS1, here is an example on how to start rosbridge that will enable communication between ROS1 and ROS2.

Note that ROS2 and ROS1 packages should always run in separate terminals to avoid mixing library paths.

Before starting the bridge, start roscore either locally, or if starting it in container, open new terminal:

```
source_ros
roscore
```

Command to start bridge in the new terminal (please mind the order of using aliases for sourcing):

```
source_ros
ros2_ws
source_ros2
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
If you are working in the group and you are all using the same network, please check [ROS_DOMAIN_ID](https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html#the-ros-domain-id-variable) in .bashrc in container. Random number should be set during the build, however it is not impossible that some of you got the same number. If that is the situatation please change it, so that your simulations do  not crash.

General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

General info on bridge can be found [here](https://github.com/ros2/ros1_bridge/blob/master/README.md) and [here](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)

## Bonus section
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool that allows you to start a tmux session with a complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal.

You don't need to write new configuration files for your projects, but some examples will use Tmuxinator. You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside the Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with the command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in Ranger.

**Htop** is a better version of `top` - command line interface task manager. Start it with the command `htop` and exit with `q`.

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i mrs_project`) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).

### TODO
- [x] Create separate repo for icuas_competition ros2 package and rename it to relay_race
- [x] Clean the docker, are ros1, MPC example, ros1_bridge needed? 
- [x] Add charging node
- [ ] Add sdf file of a space
- [ ] Change https to ssh
- [ ] One alias for sourcing
