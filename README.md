# GPpackages


in order to use gazebo services you need to run it using roslaunch instead of the script provided in mdk by consequential
to do this add this two line to the .bashrc file to make available to gazebo the models and the plugins needed to 
properly set up a miro .world
copy a file .word in a directory worlds in the miro package and use a launch file as the one in this repo
export GAZEBO_MODEL_PATH=/home/emaro_lab/mdk/sim/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/home/emaro_lab/mdk/bin/deb64:${GAZEBO_PLUGIN_PATH}

the home directory as well as the ros workspace may be different so set them up correctly

to launch the gazebo simulation with miro model sim01 and enough space to move around obstacle run
roslaunch miro miro_gazebo_ros.launch 

after the gazebo enviroment is ready to test the obstacle avoidance behaviour run
roslaunch miro obstacle_avoidance.launch

NOTES: at the beginning we used the script in mdk/sim/gazebo --> launch_sim.sh to set up the gazebo enviroment:
this led to two problems: first the gazebo ros services were not provided since we didn't use the ros package gazebo_ros. Second there were moments whene the odometry seemed out of synch, at the beginning it seemed like the robot wheels were slipping. But this phenomena did not happen when gazebo_ros pacjkage was use so we think there might be some problems in msg synchronization when not using the gazebo_ros package.  

