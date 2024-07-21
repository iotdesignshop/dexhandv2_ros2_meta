# dexhand_ros2_meta
Metapackage for DexHand V2 ROS 2 Packages. Used to manage the collection of ROS 2 packages and environment for the DexHand V2 humanoid robot hand.

## Overview
The DexHand V2 is a dexterous humanoid robot hand intended for research applications in dexterous manipulation. It is the second generation hand design in our line up. You can visit our homepage at http://www.dexhand.com.  

At the moment, the software stack for the DexHand V2 is open source, while the hardware design is not. This allows you to use the tools and simulated DexHand environment to determine if the DexHand V2 may be a useful tool for you, but ultimately hardware is a commercial package.

This package, will help you to gather together a workspace and environment for building and running the ROS 2 packages to control either a real or simulated DexHand V2.

### DexHand V1 and Open Source Hardware
If you are looking for the fully open-source version of our DexHand (V1), you can visit our project webpage to get a bit more context on the project and various packages and components: http://www.dexhand.org. The V1 hand is available with full mechanical and software under an open source license for you to build and learn.

## Pre-Requisires and Dependencies

### Notes on Fresh Linux Installs

There are a few gotchas that may come up if you have freshly installed Linux and you're about to install ROS and the other tools. While these are not directly problems with this package, they come up frequently enough that we wanted to add a couple notes to avoid pitfalls.

#### Ensure python3-pip is Installed on Your System

Pip does not seem to come installed by default on Ubuntu and this will cause problems with ROS package dependencies and builds. To install

```sudo install python3-pip```

### ROS 2 Humble Installation

Currently, we build and test our ROS 2 packages against [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on Ubuntu Linux 22.04. You may be able to get the packages running on different versions of ROS, or on different platforms but we haven't tested those and likely won't be able to provide a ton of advice or troubleshooting help.

### DexHand Hardware

DexHand hardware isn't an absolute requirement, as we've created basic simulation packages in RVIZ that will allow you to evaluate many of the tools and the function of the DexHand V2 without hardware. The software suite is open source under a MIT license.

## ROS 2 Package Overview

This metapackage depends on a number of ROS 2 packages that we maintain on GitHub. This list provides links and an overview for each package, but more detailed information about the packages is available in the README files contained within each repo.

* [**dexhandv2_description**](https://github.com/iotdesignshop/dexhandv2_description): URDF files, and RVIZ2 launch files for viewing the DexHand V2 URDF in RVIZ2. 

* [**dexhandv2_control**](https://github.com/iotdesignshop/dexhandv2_control): This package provides low-level control (subscribes to DexHand V2 messages/commands, and re-publishes to ROS topics) as well as a high-level control (virtualizes servos so that you interact with servo objects, and/or provides approximated joint angle control)


## How to Use This Meta Package

### Concept

Basically, this project is a ROS 2 Workspace that you should be able to clone and build in order to build the set of ROS 2 packages needed to control the DexHand V2.

### Usage

First, you clone this repo to your local machine:

```
git clone https://github.com/iotdesignshop/dexhandv2_ros2_meta.git
```

Then change to that directory:
```
cd dexhandv2_ros2_meta
```

Then update the git submodules:
```
git submodule update --init --recursive
```

Source your base ROS 2 environment:
```
source /opt/ros/humble/setup.bash
```

Then, install all the dependencies using rosdep:
```
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

**Note: if you haven't run rosdep on your system before, you may need to initialize it as follows:**
```
sudo rosdep init
rosdep update
```

Finally, you should be able to build the packages using colcon:
```
colcon build
```

Once the build is complete, source the environment and you should be ready to go:
```
source install/setup.bash
```

At that point, you should be ready to use the packages and the launch files contained within them, see the next section for some examples.


## Some Example Use Cases

These examples below all launch hand visualizations and simualtions using the RVIZ2 tool from ROS 2. Additional notes are provided on how to connect to and use DexHand hardware below. You should be able to try out all the demos in this section without a DexHand device, so feel free to experiment!


### Launching RVIZ2 with the Joint State Publisher GUI

This launches RVIZ2 with the DexHand URDF as well as the Joint State Publisher GUI allowing you to experiment with all of the DOF's in the DexHand in an interactive manner. This functionality is provided by the [dexhandv2_description package](https://github.com/iotdesignshop/dexhandv2_description). 


```
ros2 launch dexhandv2_description display.launch.py
```

## Using DexHand Hardware

Information on the purchase of DexHand V2 hardware is available at http://www.dexhand.com.

The [**dexhandv2_control**](https://github.com/iotdesignshop/dexhandv2_control) ROS 2 package provides everything needed to connect to a DexHand V2 device via USB. There are two nodes provided by the package:

- **native_messaging Node** - Provides a mapping between ROS 2 topics and the native messaging system of the DexHand SDK. This is the fastest and most direct method for controlling DexHand V2 hardware via ROS.
- **high_level_control Node** - Provides an abstraction layer which allows you to communicate with virtual Servo objects, or via a joint angle control system which subscribes to ROS 2 joint_state messages.

All of these options and usage of the hardware control nodes are covered in detail in the README found in the [**dexhandv2_control**](https://github.com/iotdesignshop/dexhandv2_control) repo.


## MANUS VR Glove Support

We recently added support for the MANUS VR Glove SDK in the DexHand packages. This is an optional install as it does require authenticated access to download and install the MANUS SDK, so we didn't want to just assume that by default. However, if you do have MANUS VR Gloves and an account, you can use them to control the DexHand V1/V2 Simulator and Hardware. 

[<img src="https://github.com/iotdesignshop/dexhand_ros2_meta/assets/2821763/1aa798ca-7fbe-4b78-8a0f-3a43b02b361b" width="600">](https://www.youtube.com/watch?v=Wlhi0QKMN1o)

Check out our example above of the MANUS VR Gloves controlling our RVIZ 2 based sim for the DexHand V1.


### Installation Process

We assume you have already completed the installation process described at the top of this README for setting up a workspace with the DexHand packages. The MANUS setup occurs in the same folder and adds a few packages. 

#### Adding Manus ROS Nodes to Your Workspace

Change back into the directory where you cloned the dexhand_ros2_meta package, and cd into the /src folder there:

```
cd <Your Dexhand V2 Workspace Path>
cd src
```

Clone the manus_ros2 repo and dexhandv2_manus repos into the /src folder:
```
git clone https://github.com/iotdesignshop/manus_ros2.git
git clone https://github.com/iotdesignshop/dexhandv2_manus.git
git clone https://github.com/iotdesignshop/dexhand_manus_gui.git
```

#### Installing the Manus SDK

After that, you will need to download and add the Manus SDK to the `/ext` folder of the `manus_ros2` project. This requires a Manus account and login, which you can create on the Manus site. You likely already have one if you've installed Manus Core on your PC. 

https://www.manus-meta.com/resources/downloads/overview

Once installed, you should have the Manus SDK in the /ext folder similar to this:

```
manus_ros2/
├── CMakeLists.txt
├── ext
│   ├── MANUS_Core_2.3.0.1_SDK
│   └── readme.txt
├── LICENSE
├── package.xml
├── README.md
└── src
    ├── manus_ros2.cpp
    ├── SDKMinimalClient.cpp
    └── SDKMinimalClient.hpp
```

**Note: If you haven't set up the dependencies for the Manus Linux SDK previously, you will also need to follow these instructions:https://www.manus-meta.com/knowledge-articles/software-core-2-3-linux-sdk**

#### Building the Workspace

Once installation is complete, you can switch to your workspace, build the new packages, source the environment and you should be ready to go:
```
cd <Your Workspace Path>
colcon build
source install/setup.bash
```

#### Launching the Simulator With Manus Input

To launch the Manus nodes along with a simulation of the DexHand for testing, you can run:
```
ros2 launch dexhandv2_manus simulation.launch.py
```

__NOTE: The Manus SDK requires Manus Core to be running on a Windows PC on the same network as the client__. We have had pretty good results running Linux in a Virtual Machine on a Windows PC, and that may be a good way to get started. Manus plans to release a Linux version of Manus Core in the future, and we'll update the instructions when that happens.__



