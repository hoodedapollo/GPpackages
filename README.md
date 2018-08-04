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

Now based on what you want to do there are different possibilities:
* choose wheter to run a simulation of miro or o use the real robot
* choose wheter to guide Miro with the smartwatch alone or with obstacle avoidance and human influence 

In order to simplify the task of setting up the whole enviroment.launch files are available as  described in the following sections

#### Gazebo Enviroment Setup

This step is required only if you want to use a simulated MiRo robot.

To open gazebo with a model of miro named sim01 (robot_name parameter) and enough free space to manouver 
```
roslaunch miro_gazebo_ros miro_gazebo_ros.launch
```

#### Enable Gesture Based Control  
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

Instead if you want to enable the gesture based smartwatch control compatible with the obstacle avoidance behaviour you should run

```
roslaunch gb_control gb_control_to_coordination.launch
```


#### Enable Autnomous Obstacle Avoidance Behaviour
If in the previous step you chose the second command now run

```
roslaunch miro_oa_behaviour obstacle_avoidance_to_coordination.launch
```

parameters in the launch file:
* robot_name 
    * sim01
    * rob01
At this point miro will follow your instructions and will try to avoid obstacles autonomously provided that you will otherwise ;)

### Parameters Explanation
* robot_name 
    * sim01 is the name of the simulated robot in gazebo
    * rob01 is the name of the real miro


### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

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

