#!/bin/bash

cd ~/robotic
xterm -hold -e "rosrun follow_me decision_node" &
xterm -hold -e "rosrun follow_me object_detector_node" &
xterm -hold -e "rosrun follow_me robot_moving_node" &
xterm -hold -e "rosrun follow_me translation_node" &
xterm -hold -e "rosrun follow_me obstacle_detector_node" &
xterm -hold -e "rosrun follow_me rotation_node" &
