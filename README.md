# sawback
<p align="center">
  <img src="/sawback_manipulation/media/ridgeback_pick.gif" width="350" height="300"/>
  <img src="/sawback_manipulation/media/pick_rviz.gif" width="350" height="300"/>
</p>

# Table of Contents
- [Overview](#Overview) </br>
- [Packages](#Packages) </br>
- [Getting Started](#Getting-Started) </br>
- [Manipulation and Grasping Demo](#Manipulation-and-Grasping-Demo)</br>

# Overview
A supplementary repository to [nu_ridgeback](https://github.com/bostoncleek/nu_ridgeback). This repository contains the tools
for manipulation using the Sawyer on board the Ridgeback.

<p align="center">
  <img src="/sawback_manipulation/media/nurb1.jpg" width="400" height="400"/>
</p>

# Packages
- [sawback_description](https://github.com/bostoncleek/sawback/tree/master/sawback_description): Sawback URDF
- [sawback_manipulation](https://github.com/bostoncleek/sawback/tree/master/sawback_manipulation): Pick and Place pipeline
- [sawback_moveit_config](https://github.com/bostoncleek/sawback/tree/master/sawback_moveit_config): Moveitcpp configuration and SRDF
- [sawback_msgs](https://github.com/bostoncleek/sawback/tree/master/sawback_msgs): Manipulation messages

# Getting Started
If you have completed the Getting Started section of the [nuridgeback_robot](https://github.com/bostoncleek/nu_ridgeback/tree/master/nuridgeback_robot) package then skip steps 2 and 3. Otherwise complete the step bellow to setup the workspace.

1. Grasp pose detection
  Install the [grasp pose detection](https://github.com/atenpas/gpd#install) library.
  On the Ridgback's computer this is installed in `~/Libraries`.

  - Important: The grasping library must be install with `sudo make install` so this package can link against it.
  - Important: Update the absolute file path to the `gpd/models/lenet/15channels/params/` directory on line 49 in `sawback_manipulation/config/gpd_config.yaml`.


2. Create the workspace and close the relevant packages
  ```
  mkdir -p ~/sawback_ws/src
  cd ~/sawback_ws/src
  wstool init .
  wstool merge -t . https://github.com/bostoncleek/sawback/blob/master/sawback.rosinstall
  wstool update -t .
  ```

3. Build the workspace
  ```
  cd ~/sawback_ws
  catkin init
  catkin build -DMCAKE_BUILD_TYPE=Release
  ```

# Manipulation and Grasping Demo
To launch the manipulation pipeline see [sawback_manipulation](https://github.com/bostoncleek/sawback/tree/master/sawback_manipulation). To see the full mobile manipulation sequence see this [video](https://youtu.be/iLyqu9EoNtY).
