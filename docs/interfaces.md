# Interface
This document decribes interfaces between the julia framework and the ROS integration code.

## General Ideology
There are 3 modular components to the framework:
1. Julia framework - handles the computation of actuation given state, observation
2. ROS communication interface - serve as bridge between Julia and ROS, as well as handling communciation among different cars
3. ROS simulator/real robot - handles the simulation

## Interface design
### Julia
1. init_socket() initializes and returns the socket communication. Since this socket handles specifically 1-1 communciation, @async is not needed
2. init_framework(xs: MVector{N, X}, us: ..., zs: ...) where N, X, U, Z initialiazes the entire framework with the given initial state. It will need to return a controller that takes in the states and output actuation
3. run_framework(comm: Socket, xs, us, zs, controller) takes in information of all of the above and start the framework.

At every timestep, the framework needs to send actuation and time, where actuation is computed from the controller and the given states. It will then accepct the state from ROS communication and repeat the proceess.

If the communication is lost, halt all running process.

As time by default starts from 0, when the frameworks starts it will need to keep track of the current time and subtract real time difference. 

### ROS communication interface
1. recieveCommunication() Recieves and handles the communication for state. It needs to forward state to all other robots, who then will accept in a different thread. 
2. step() - in faster than real time sinulation: send state and the current issue_time (keep in an internal state tracker). In real time simulation/real env: send state


Unfortunatly this is where simulaiton and real difference will cause issue - this will cuase issue as version difference between ROS message will in turn cause compilation issue. - IFDEF can be used to get around - though this is far from elegant.

3. sendCommunication() send the latest observation from the environment. This needs to happen at the front of the loop as that is the only way to ensure little processing latency.

## ROS simulator/real rebot
WIP

