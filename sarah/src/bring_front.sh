#!/bin/bash
rostopic pub -1 /waist_controller/command std_msgs/Float64 -- 0 &
sleep 0.5
rostopic pub -1 /wrist_controller/command std_msgs/Float64 -- 0 &
sleep 0.5
rostopic pub -1 /elbow_controller/command std_msgs/Float64 -- 1.4 &
sleep 0.5
