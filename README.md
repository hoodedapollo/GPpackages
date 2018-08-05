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

### Prerequisites 

You must have a smartwatch paired with a smartphone which is the bridge between the smartwatch and the ros master running on your computer.
Follow the instructions reported in this repository (link to carfi repo)

Follow the instructions at consequential robotics to set up your workstation to work with miro. Especially use the mdk link as sugested.

Since we will use the gazebo_ros package to run the gazebo simulation instead of the script file provided by consequential robotics(gazebo services for debug and better comunication synch) 
add this two line to the .bashrc file to make available to gazebo the models and the plugins neede to
properly set up a miro .world
export GAZEBO_MODEL_PATH=/home/user/mdk/sim/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/home/user/mdk/bin/deb64:${GAZEBO_PLUGIN_PATH}

change user with your user name and deb64 with your distribution between those available under the mdk/bin directory


### ROS enviroment setup
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

This repository contains all the packages needed to run the project in simulation as well as with a real Miro.

Create a catkin workspace and clone all the packages in the src folder

```
$ mkdir -p catkin_ws/src
$ cd catkin_ws && catkin_make
$ cd src && git clone this repo
$ cd .. && source devel/setup.bash
```
From now on we will assume that your catkin workspace is called catkin_ws.  

### Run the project

mosquitto must be running on your PC for the birdge to work 
```
$ mosquitto
```
Make sure that the IP in the IMU_stream app on the  smartphone is correct

Open the IMU_stream app on smartwatch 



Now put on the smartwatch and run

```
roslaunch coordination oa_gb_simple_coordination.launch
```

A gazebo enviroment as the one in the picture will show up and rviz visualizer will show the odometry data.
Now you should be able to control miro with your smartwatch

Add as many obstacle as you like and see how miro behaves ;)

### Architecture Breakdown

This project is composed of different parts.

The command 

```
roslaunch coordination oa_gb_simple_coordination.launch
```

is responsible to run all the different parts in one shot, but you can also run them separately.
This section gives an insight on the different parts tha compose the overall system
* Gazebo Enviroment Setup
* Gesture Based Control
* Obstacle Avoidance Behaviour
* Coordination

#### Gazebo Enviroment Setup

To open gazebo with a model of miro named sim01 and enough free space to manouver 

```
roslaunch miro_gazebo_ros miro_gazebo_ros.launch
```

#### Enable Gesture Based Control Alone

To enable the gesture based smartwatch control alone 

```
roslaunch gb_control gb_control_to_miro.launch
```

main parameters in the launch file:
* robot_name 
    * sim01
    * rob01
* gb_control_type
    * attitude
    * linear

At this point you should be able to control Miro with your smartwatch

#### Enable Obstacle Avoidance alone

To enable only the obstacle avoidance behaviour run
```
roslaunch miro_oa_behaviour obstacle_avoidance_to_miro.launch
```

main parameters in the launch file:
* robot_name 
    * sim01
    * rob01

#### Coordination

To run the overall architecture now you do not publish command directly to miro as in the above
sections but you provide them to the cordiation node.
This is what the oa_gb_simple_coordination.launch file does.

### Parameters Explanation

Explanation of most of the parameters you will encounter in the launch files

* robot_name 
    * sim01 is the name of the simulated robot in gazebo
    * rob01 is the name of the real miro

* control parameters for onstacle avoidance

* rate:is the rate at which the nodes which use this parametr run

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc

