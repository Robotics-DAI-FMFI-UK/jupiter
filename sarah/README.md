# A transport robot carrying bricks for you

## About the project

You can control the robot with the joystick (while pressing RB) to drive through the room. 
It will automatically map the room and remember obstacles. Once the map is sufficiently 
prepared, the robot can be steered to the start point. By pressing B, it remembers this 
position. (Make sure that log prints position, sometimes you have to press harder.
Then you can steer the robot with the joystick to follow you. Once you are at the place where
you have the bricks you want to carry, you can press B again and it will stretch out it's hand
(if the arm is properly initialized and in the right default position). You can give it the 
cube and it will put it on it's base. Then it will rush to the start pose, put down the 
cube (ideally, just take it off), and return to where you are waiting. By pressing B repeatedly 
you can bring multiple bricks back and forth. You can also steer the robot to another end point
before pressing B again, to set this as the new end position.

## How to start the program
To run the program, start the robot (+arm), connect to the robot and open up two terminals.
If you do not wish to use the arm, because of it's low reliability, comment out 
```<include file="$(find rchomeedu_arm)/launch/arm.launch"/>```
in bricks.launch. It will not break the program, it will only have one button press that 
does nothing. Go into jupiter/sarah in both terminals.
In one of the terminals call 
```roslaunch sarah bricks.launch```
and make sure that everyone starts as it should. (Especially the arm or you will have
millions of overload errors within seconds)
Then call 
```rosrun sarah run.py```
in the other terminal. You can now drive around with the joystick and explore the map.
Once you have found your start pose (the pose all items should be brough to), you can press B
and check that the coordinates get printed in the terminal.
After that you can use the robot for transport as described above.

## See the robot drive on YouTube



