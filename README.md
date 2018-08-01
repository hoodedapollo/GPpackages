# GPpackages

in order to use gazebo services you need to run it using rolaunch instead of the script provided in mdk by consequential

to do this add this two line to the .bashrc file to make available to gazebo the models and the plugins needed to 
properly set up a miro .world
copy a file .word in a directory worlds in the miro package and use a launch file as the one in this repo
export GAZEBO_MODEL_PATH=/home/emaro_lab/mdk/sim/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/home/emaro_lab/mdk/bin/deb64:${GAZEBO_PLUGIN_PATH}

to launche the gazebo simulation with miro with enough space to move around obstacle run
roslaunch miro my_miro.launch 
