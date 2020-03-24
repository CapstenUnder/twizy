# TwizyProject
#### Github repository for bachelor thesis

Starting the car requiers the correct codes on both micro controller in in the front of the car,
contact the group members from 2018 for more info. There are some elctrical problems in the car 
(from previus projects), if the car dashnoard says "stop" the car will not drive and needs a restart 
if it does not go away. After every run the power cabel (USB connecting on the left side)for the
throttle will need to be disconnected, otherwise it will store last value and drive unwanted. 

To run with wasd
0 Connect laptop to CAN
1 Run roscore
2 Run rosrun twizy control_node
3 Run rosrun twizy testreader.py
