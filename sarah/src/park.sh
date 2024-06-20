#!/bin/bash
rostopic pub -1 /hand_controller/command std_msgs/Float64 -- -0.2 &
sleep 0.5
rostopic pub -1 /elbow_controller/command std_msgs/Float64 -- 1.7 &
sleep 0.5
rostopic pub -1 /wrist_controller/command std_msgs/Float64 -- 1.8 &
sleep 0.5
rostopic pub -1 /shoulder_controller/command std_msgs/Float64 -- -1.9 &
