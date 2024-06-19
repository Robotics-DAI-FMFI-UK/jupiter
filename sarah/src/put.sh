#!/bin/bash
rostopic pub -1 /wrist_controller/command std_msgs/Float64 -- 0 &
sleep 0.5
rostopic pub -1 /shoulder_controller/command std_msgs/Float64 -- 0 &
sleep 0.5
rostopic pub -1 /elbow_controller/command std_msgs/Float64 -- 2.2 &
sleep 0.5
rostopic pub -1 /waist_controller/command std_msgs/Float64 -- -2.2 &
sleep 3
killall rostopic
rostopic pub -1 /wrist_controller/command std_msgs/Float64 -- 0.5 &
sleep 1
rostopic pub -1 /elbow_controller/command std_msgs/Float64 -- 1.95 &
sleep 1
killall rostopic
rostopic pub -1 /wrist_controller/command std_msgs/Float64 -- 1.1 &
sleep 0.5
rostopic pub -1 /shoulder_controller/command std_msgs/Float64 -- 0.1 &
sleep 0.5
rostopic pub -1 /hand_controller/command std_msgs/Float64 -- -0.2 &
sleep 0.5
killall rostopic
rostopic pub -1 /shoulder_controller/command std_msgs/Float64 -- -0.5 &
