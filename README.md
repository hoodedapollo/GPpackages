# Developemnt of a pet-like behaviour for MiRo Robot

## Introduction
The overall idea behind the project is to develop effective ways of interaction between Humans and MiRo and Miro and the environment.

### MiRo Companion Robot
MiRo is a animal-like robot developed as a prototype companion.
It was designed with a bio-inspired architecture based on the neuroscience knowledge of the mammalian brain.

### The Metaphor
In order to accomplish the goal of a pet-like behaviour for MiRo, we thought to exploit a typical Human-Pet interaction: Taking the dog for a walk.
A pet on a leash  follows his owner but at the same time it preserves his own safety avoiding collision with obstacles.
Furthermore, the pet is willing to follow his owner commands depending on its emotional state which is influenced by the interaction with the humans and the environment.

### The Problem
Basically the problem we address is to use a wearable device to guide MiRo. Its reactions to the user commands will be strongly influenced by its emotional state depending on external and internal stimuli.
While being controlled by the user, if Miro encounter an obstacle it will try to avoid it by bringing itself in a safe condition and then by go round it. The user will have the power to override the circumnavigation of the obstacle by exerting his influence trough specific gestures.
When MiRo is not under user control it will react depending on it emotional state. It's mood will be directly affected by its interaction with humans and the environment. However, if something approaches MiRo too closely it will bring itself is a safe condition by going away from the obstacle (threat).
This problem is composed of multiple aspects:
* Determine and implementing autonomous behaviour for Miro based on the interaction with the environment ( such as avoiding collision with objects)
* Define a Model for MiRo emotional state which determines his behaviour in absence of commands by the user
* Devise a Controller which adheres to the leash metaphor
* Coordinate the different behaviours by keeping in mind that in some situation the control is shared between MiRo and the user.

## Getting Started

### ROS
This project is developed using [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu):
* rosdistro: kinetic
* rosversion: 1.12.13

### Smartwatch and Smartphone Setup
In order to publish imu data from your smartwatch to ROS nodes you must have a smartwatch paired with a smartphone.
The smartphone acts as the bridge between the smartwatch and the ros master running on your computer.

Follow the instructions reported in [imu_stream](https://github.com/EmaroLab/imu_stream) to download the app for both the smartphone and the smartwatch.

### MQTT ROS Bridge

In order to succesfully run the code, you should have installed [paho-mqtt](https://pypi.python.org/pypi/paho-mqtt/1.1): clone the repository and install paho-mqtt from it. 

The suggested MQTT Broker is [Mosquitto](https://mosquitto.org/documentation/). In order to install Mosquitto on Ubuntu follow [this guide](https://www.digitalocean.com/community/tutorials/how-to-install-and-secure-the-mosquitto-mqtt-messaging-broker-on-ubuntu-16-04).

### MiRo Workstation Setup
Download the [Miro Developer kit](http://labs.consequentialrobotics.com/miro/mdk/).

Follow the instructions from Consequential Robotics [Miro: Prepare Workstation](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html) to set up your workstation to work with miro. 
Strictly follow the instructions in the Install MDK section as the following steps will rely on this.

Not necessary to make static IP for your workstation (laptop) while setting up connection with MiRo.

[You may skip the static IP part, which is described in the setup files in this repository. Also you can skip the ROS and Gazebo installation part, if you already have it on your workstation]

We use the gazebo_ros package to run the gazebo simulation, instead of the script file provided by consequential robotics.  
For this reason add this two line to the .bashrc file. This makes available to gazebo the models and the plugins provided by the Miro Developer Kit

```
export GAZEBO_MODEL_PATH=/home/user/mdk/sim/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/home/user/mdk/bin/deb64:${GAZEBO_PLUGIN_PATH}
```

change user with your user name and deb64 with your distribution from those available under the mdk/bin directory

### ROS enviroment Setup
This repository contains all the packages needed to run the project in simulation as well as with a real Miro.

Create a catkin workspace and clone all the packages in the src folder

```
$ git clone https://github.com/hoodedapollo/MiroBehaviours.git
$ cd MiroBehaviours && catkin_make
$ source devel/setup.bash
```
### Run the project

mosquitto must be running on your PC for the birdge to work 
```
$ mosquitto
```
Make sure that the IP in the IMU_stream app on the  smartphone is correct

Open the IMU_stream app on smartwatch 

OPTIONAL:
The simulated miro does not have touch senors nor led lights which are used in the emotional behaviour.
For this reason, if you want to use a real miro for the emotial behaviour you must connect it to the ROS master.

```
$ ssh root@<MIRO-IP> 
$ sudo nano ./profile
```
Insert your IP after ROS_MASTER_IP

For more detailed instructions see [MIRO: Commission MIRO](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html)

The following command will start the project

```
$ roslaunch coordination complete_launcher.launch
```

A gazebo enviroment will show up and rviz visualizer will show the odometry data.
Now you should be able to control miro with your smartwatch.

Add as many obstacle as you like and see how miro behaves

### Architecture Breakdown

This project is composed of different parts.

The command 

```
$ roslaunch coordination oa_gb_simple_coordination.launch
```

is responsible to run all the different parts in one shot, but you can also run them separately.

This section gives an insight on the different parts that compose the overall system
* Gazebo Enviroment Setup
* Gesture Based Control
* Obstacle Avoidance Behaviour
* Coordination

#### Gazebo Enviroment Setup

To open gazebo with a model of miro named sim01 and enough free space to manouver run 

```
$ roslaunch miro_gazebo_ros miro_gazebo_ros.launch
```

#### Enable Gesture Based Control Alone

To enable the gesture based smartwatch control alone run

```
$ roslaunch gb_control gb_control_to_miro.launch
```

Set this parameters in the launch file:
* robot_name 
    * sim01: if you want to control the miro in the simulated enviroment at the previous step
    * rob01: if you want to control the real Miro Robot
* gb_control_type
    * attitude: control based on a complemetary filer of the imu data
    * linear: control based directly on imu data

At this point you should be able to control Miro with your smartwatch

#### Enable Obstacle Avoidance alone

To enable only the obstacle avoidance behaviour run
```
$ roslaunch miro_oa_behaviour obstacle_avoidance_to_miro.launch
```

set this parameter in the launch file:
* robot_name 
    * sim01
    * rob01
* control parameters

This is helpful for tuning the pid for the real miro.

#### Coordination

To run the overall architecture commands are not published directly to miro as in the above
sections but they are provided to the cordiation node.
This is what the oa_gb_simple_coordination.launch file does.

##Results 

### Gesture Based linear vs attitude control
[![GestureBasedlinearvsattitudeControl](https://ibb.co/czCKC9.jpg)](https://drive.google.com/open?id=1aDZP9WhCtvIU5SouSsq_7nUBMfI9DegY)

### Gesture Based and Obstacle Avoidance Coordination test

### Autonomous Emotional Behavior

## Acknowledgments

* [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge) 
* [imu_stream](https://github.com/EmaroLab/imu_stream)
* [imu_complementary_filter](http://wiki.ros.org/imu_complementary_filter)

###Authors

* Marco Ruzzon: marco.ruzzon@mail.polimi.it
* Roberta Delrio: roberta.delrio@studio.unibo.it

