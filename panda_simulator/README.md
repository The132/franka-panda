# Panda Simulator [![Release](https://img.shields.io/badge/release-v1.0.0-blue.svg)](https://github.com/justagist/panda_simulator/tree/melodic-devel) [![ROS Version](https://img.shields.io/badge/ROS-Melodic-brightgreen.svg?logo=ros)](https://ros.org/) [![Python 2.7](https://img.shields.io/badge/python-2.7-blue.svg?logo=python)](https://www.python.org/downloads/release/python-275/) [![Build Status](https://travis-ci.com/justagist/panda_simulator.svg?branch=melodic-devel)](https://travis-ci.com/justagist/panda_simulator)

[![franka_ros_version](https://img.shields.io/badge/franka_ros-v0.6.0,%20v0.7.0,%20v0.7.1-blue.svg)](https://github.com/frankaemika/franka_ros) [![franka_ros_interface_version](https://img.shields.io/badge/franka_ros_interface-v0.6.0,%20v0.7.1-yellow.svg)](https://github.com/justagist/franka_ros_interface)

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*Franka ROS Interface*][fri-repo] package.

**:sparkles: _NEW_ :sparkles:: ROS Noetic now supported. See '[noetic_devel](https://github.com/justagist/panda_simulator/tree/noetic-devel)' branch.**

## Features

- Customisable low-level *controllers* (joint position, velocity, torque) available that can be controlled through ROS topics (including position control for gripper) or [Python API][fri-repo].
- Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.
- The [*Franka ROS Interface*][fri-repo] package (which is a ROS interface for controlling the real Panda robot) can also be used with the panda_simulator, providing direct sim-to-real code transfer. The [PandaRobot][pr-repo] package which provides simplified python API for the robot can also be used for direct sim-to-real code transfer, as well as for performing real-time kinematics and dynamics computation.
- Supports MoveIt planning and control for Franka Panda Emika robot arm and Franka Gripper, as well as smooth motion control using customisable ROS JointTrajectoryAction service.

*For a simple bare-bone Gazebo simulator created using inbuilt Gazebo ROS controllers and transmission interfaces, see [Gazebo Panda](https://github.com/justagist/gazebo_panda).*

*See [version log](https://github.com/justagist/panda_simulator/blob/melodic-devel/versionLog.md) for details about new feature updates.*

### Control and Monitor robot using Python API

Python API: [Franka ROS Interface][fri-repo], [PandaRobot][pr-repo]

  ![vid](https://github.com/justagist/franka_ros_interface/blob/master/assets/panda_robot_demo.gif)
 Watch video [here](https://youtu.be/4bEVysUIvOY)

  ![vid](https://github.com/justagist/franka_ros_interface/blob/master/assets/panda_simulator.gif)
 Watch video [here](https://www.youtube.com/watch?v=NdSbXC0r7tU)

### Control using MoveIt

  ![vid](https://github.com/justagist/franka_ros_interface/blob/master/assets/moveit_demo.gif)
 Watch video [here](https://youtu.be/a_HEmYzqEnk)

### Installation

ROS Melodic (noetic-devel branch): [![Build Status](https://travis-ci.com/justagist/panda_simulator.svg?branch=noetic-devel)](https://travis-ci.org/justagist/panda_simulator)

ROS Melodic (melodic-devel branch): [![Build Status](https://travis-ci.com/justagist/panda_simulator.svg?branch=melodic-devel)](https://travis-ci.com/justagist/panda_simulator)

ROS Kinetic (kinetic-devel branch; *not maintained*): [![Build Status](https://travis-ci.com/justagist/panda_simulator.svg?branch=kinetic-devel)](https://travis-ci.com/justagist/panda_simulator)

#### Dependencies

- `pip install -r requirements.txt #(to install numpy and numpy-quaternion)`
- *libfranka* (`apt install ros-${ROS_DISTRO}-libfranka` or [install from source][libfranka-doc])
- *franka-ros* v0.7.1 (`apt install ros-$ROS_DISTRO-franka-ros` or [install from source][libfranka-doc]). *Make sure to use the [release version](https://github.com/frankaemika/franka_ros/tree/902fdbba0f7c6036a84a688712a454b9e622863b) if building from source. (`git checkout 902fdbb` from the cloned franka_ros github repo.)* **Note:** For using simulator with older version of *franka_ros* (v0.6.0), the last supported version of Panda Simulator was in commit [04dd906](https://github.com/justagist/panda_simulator/tree/04dd906f1923aa286fc5a1593b04bd35bfba78ee). The newer features and functionalities may not be available in the older version.
- *panda_moveit_config* (`apt install ros-$ROS_DISTRO-panda-moveit-config` or [install from source](https://github.com/ros-planning/panda_moveit_config))
- Most of the other basic dependencies can be met by running the following `apt-get` command: `apt install ros-$ROS_DISTRO-gazebo-ros-control ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools`.

The following dependencies can be installed using the `.rosinstall` file (instructions in next section: [Building the Package](#building-the-package)).

- [*Franka ROS Interface*][fri-repo] ([`v0.7.1`](https://github.com/justagist/franka_ros_interface/tree/v0.7.1) branch)
- [*franka_panda_description*][fpd-repo] (urdf and model files from *panda_description* package modified to work in Gazebo, with the custom controllers, and more realistic dynamics parameters)
- [*orocos-kinematics-dynamics*](https://github.com/orocos/orocos_kinematics_dynamics)

**NOTE**: The franka_panda_description package above has to be independently updated regularly (using `git pull`) to get the latest robot description, visual and dynamics parameters.

#### Building the Package

1.Clone the repo:

```bash
    cd <catkin_ws>/src
    git clone -b melodic-devel https://github.com/justagist/panda_simulator
```

Steps 2 and 3 can be automated by running `./build_ws.sh` from `<catkin_ws>/src/panda_simulator`.

2.Update dependency packages:

```bash
    wstool init
    wstool merge panda_simulator/dependencies.rosinstall
    wstool up

    # use old ros-compatible version of kdl
    cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc
    cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

3.Once the dependencies are met, the package can be installed using catkin_make:

```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)
    source devel/setup.bash
```

### Docker Build (experimental!)

**Requires [nvidia-docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)). Gazebo and RViz may not work without nvidia-docker.**

- To build the docker image of the package, run `docker build docker/ -t ps_${ROS_DISTRO}:v1.0.0`, or pull built image from github (`docker pull docker.pkg.github.com/justagist/panda_simulator/ps_${ROS_DISTRO}:v1.0.0`).
Note: Even when using the docker image, this repository has to be cloned on to the host machine.
- To run the built image interactively, run the script `./run_docker.sh` from the cloned repository. The container starts in a catkin workspace (directory location in host machine: `$HOME/.panda_sim_${ROS_DISTRO}_ws`). The host's home directory is also mounted in the container for access to `.ros/` and for making the catkin workspace writable. To see and modify other mounted volumes, go through the `run_docker.sh` file.
- When running for the first time, the catkin workspace has to be built (`cd src/panda_simulator && ./build_ws.sh`).
- If everything was successfully built in the previous step, you should be able to run the simulator (see [Usage](#usage) section below).

Any edits made to the host directory will be reflected in the docker container (and vice-versa). You can also run and build other ROS nodes and packages without having any ROS installation on the host machine.

### Usage

The simulator can be started by running:

```bash
    roslaunch panda_gazebo panda_world.launch # (use argument load_gripper:=false for starting without gripper; see other available arguments in launch file)
```

This exposes a variety of ROS topics and services for communicating with and controlling the robot in simulation. The robot can also be controlled using the [Franka ROS Interface](https://github.com/justagist/franka_ros_interface) package and/or [PandaRobot][pr-repo] APIs.

**NOTE: The above roslaunch command does not start the moveit server automatically. If using Panda Simulator in ROS Melodic/Noetic environment, the moveit server now has to be started manually by running the following command in a new terminal:**

```bash
    roslaunch panda_sim_moveit sim_move_group.launch # (use argument load_gripper:=false for starting without gripper
```

For known errors and issues, please see [Issues](#known-issues) section below.

#### Demos

To run these demos, launch the simulator first: `roslaunch panda_gazebo panda_world.launch`. The following demos can then be tested:

- Moveit Demo: The moveit server must be running (see [usage](#usage)). Run `roslaunch panda_simulator_examples demo_moveit.launch` to run a demo for testing the moveit planner interface with the simulated robot. This script starts a moveit RViz GUI for motion planning and terminal interface for modifying planning scene.

- Task-space control using Franka ROS Interface (or PandaRobot) API: Run `roslaunch panda_simulator_examples demo_task_space_control.launch` to run a demo showing the task-space control. By default, the demo uses the (Franka ROS Interface) API to retrieve state information, and to control it using torque control (see [script](panda_simulator_examples/scripts/task_space_control_with_fri.py)).

- Task-space control using ROS topics directly: Another script demonstrating the same functionality without using the Franka ROS Interface API, and only the ROS topics from the simulation is also [provided](panda_simulator_examples/scripts/task_space_control_using_sim_only.py). This can be run interactively by running `roslaunch panda_simulator_examples demo_task_space_control.launch use_fri:=False`.

- API usage demo: Another (much simpler) demo ['move_robot.py'](panda_simulator_examples/scripts/task_space_control_using_sim_only.py) is provided demonstrating (i) controlling the robot in the joint space, (ii) retrieving state information of the robot.

##### Task-space Impedance Control Demo

  ![vid](assets/ts_demo.gif)
 Watch video [here](https://youtu.be/a_HEmYzqEnk)

#### Some useful ROS topics

##### Published Topics

| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/custom_franka_state_controller/robot_state* | gravity, coriolis, jacobian, cartesian velocity, etc. |
| */panda_simulator/custom_franka_state_controller/tip_state* | end-effector pose, wrench, etc. |
| */panda_simulator/joint_states* | joint positions, velocities, efforts |

##### Subscribed Topics

| ROS Topic | Data |
| ------ | ------ |
| */panda_simulator/motion_controller/arm/joint_commands* | command the robot using the currently active controller |
| */panda_simulator/franka_gripper/move* | (action msg) command the joints of the gripper |

Other topics for changing the controller gains (also dynamically configurable), command timeout, etc. are also available.

#### ROS Services

Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient (via the same topics as the real robot and [*franka_ros*][franka-ros]).

### Known Issues

1. `[ERROR] Exception while loading planning adapter plugin 'default_planner_request_adapters/ResolveConstraintFrames` in melodic. This can be [safely ignored](https://github.com/ros-planning/moveit/issues/1655).

2. `Error in REST request` error message when starting Gazebo. This also can be safely ignored, or fixed by following the instructions [here](https://answers.gazebosim.org/question/25030/gazebo-error-restcc205-error-in-rest-request/?answer=25048#post-id-25048).

3. ~Gripper control and model definition is not completely developed, and gripper control may not produce the required performance.~ *Update: robot and gripper model definitions have now been improved in the [franka_panda_description][fpd-repo] package*.

4. Gravity compensation when using velocity or torque controller with gripper is not very good. This is bypassed by deactivating simulator gravity by default (see [`panda.world`](panda_gazebo/worlds/panda.world)).

### Version Update

Check [versionLog.md](https://github.com/justagist/panda_simulator/blob/melodic-devel/versionLog.md).

## Related Packages

- [*Franka ROS Interface*][fri-repo] : A ROS API for controlling and managing the Franka Emika Panda robot (real and simulated). Contains controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [*PandaRobot*][pr-repo] : Python interface providing higher-level control of the robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).
- [*Gazebo Panda*](https://github.com/justagist/gazebo_panda): A simple bare-bone gazebo simulator using in-built gazebo controllers and transmissions. No custom controllers or interfaces.

The [*Franka ROS Interface*][fri-repo] package provides Python API and interface tools to control and communicate with the robot using the ROS topics and services exposed by the simulator. Since the simulator exposes similar information and controllers as the *robot_state_controller_node* of the [*Franka ROS Interface*][fri-repo], the API can be used to control both the real robot, and the simulated robot in this package, with minimum change in code.

### License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2021, Saif Sidhik

If you use this software, please cite it using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747459.svg)](https://doi.org/10.5281/zenodo.3747459).

   [fri-repo]: <https://github.com/justagist/franka_ros_interface>
   [fpd-repo]: <https://github.com/justagist/franka_panda_description>
   [pr-repo]: <https://github.com/justagist/panda_robot>
   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://frankaemika.github.io/docs/franka_ros.html>
