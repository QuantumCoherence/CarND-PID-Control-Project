# PID Controller Project
## GitHub Repo
[CarND-Controller-Project-Project](https://github.com/QuantumCoherence/CarND-PID-Control-Project)


This project was completed on Ubuntu 16.04.

For both github repo links, the ecbuild subfolder contains all ECLIPSE project files, including compiled binary.

The build subfolder contains all the binaries and the makefile for compiling the project at the prompt with no formal IDE. 

**Compiling on your system: CMakelists.txt**

Use the CMakeLists.txt in the repo root folder to create the make file and compile on your system.




### PID Parameters Notes	

####Parameters and Performance

Control Parameters - Control in this context means SteerAngle control

P - Proportional Control Response
The larger the CTE error, that is the distance from the ideal trajectory, the stronger the steering response.

D - Linear Response to the error change rate
The faster the error change, the stronger the response. This error response helps dampening the overall response to a perturbation from the ideal trajectory, hence enhanicng stabilty and performance.

I - Linear response to constant error, or to the infinite wave length component of a perturbation.
This error response eliminates any lingering bias from the desired set point. In this particular case it makes sure the final CTE error is zero (when no perturbations are present). This for instance helps eliminating steering bias or drift due to tire defects of real world vehicles.

Performance
If speed wasn't a factor, the above would work so well!
The P parameter defines how fast the system responds to a perturbation. If the paramter is set to a too small value, a tight turn at moderate speed will cause the vehicle to overshoot and perhaps even go off the road.
A higher value will keep the vehicle on the road in a tight turn, but will amplify the response to simple repeated perturbations on a straight trajectory. This is due the frequency response inherent in the system. A correclty set P parameter will casue the system response to oscillate around the set point. 

The derivative response will dampen the initial response but only for a given frequency response, which will however change with changing speed, hence choosing a set of paramters always implies a trade off between stability and performance of the controller.
Giving up on response time, that is on how tight a turn the system will handle without much overshooting, will get a more stable trajecotry, but will also limit the maximum speed the controller is able to handle without becoming unbounded.


####Expected Performance on third party computers
Given the frequency response depends amon others on the processing power of the computer system, it is well possible that the performance of this code, on  the test party system, might well be better or worse than displayed in the video below.
The system on which this code was tested is close to ancient ... so it's likley that on a third party testing this code might perform better, meaning altering the max_speed with a higher value parameter will get a stable vehicle control response at faster speed than displayed here, 

####Video and Images

**Video**
Download in ziped form, gunzip and play.
[PID Loop]()

**Images**

####Tuning Paramters

####Coding Notes

