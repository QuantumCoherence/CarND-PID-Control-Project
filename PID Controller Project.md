# PID Controller Project
## GitHub Repo
[CarND-PID-Controller-Project](https://github.com/QuantumCoherence/CarND-PID-Control-Project)


This project was completed on Ubuntu 16.04. 

For both github repo links, the ecbuild subfolder contains all ECLIPSE project files, including compiled binary.

The build subfolder contains all the binaries and the makefile for compiling the project at the prompt with no formal IDE. 

**Compiling on your system: CMakelists.txt**

Use the CMakeLists.txt in the repo root folder to create the make file and compile on your system.


## PID Project Notes	

### Parameters and Performance

##### Control Parameters - Control in this context means SteerAngle control

*P - Proportional Control Response*

The larger the CTE error, that is the distance from the ideal trajectory, the stronger the steering response.

*D - Linear Response to the error change rate*

The faster the error change, the stronger the response. This error response helps dampening the overall response to a perturbation from the ideal trajectory, hence enhanicng stabilty and performance.

*I - Linear response to constant error, or to the infinite wave length component of a perturbation*

This error response eliminates any lingering bias from the desired set point. In this particular case it makes sure the final CTE error is zero (when no perturbations are present). This for instance helps eliminating steering bias or drift due to tire defects of real world vehicles.

#### Performance

If speed wasn't a factor, the above would work so well!
The P parameter defines how fast the system responds to a perturbation. If the paramter is set to a too small value, a tight turn at moderate speed will cause the vehicle to overshoot and perhaps even go off the road.
A higher value will keep the vehicle on the road in a tight turn, but will amplify the response to simple repeated perturbations on a straight trajectory. This is due to the frequency response inherent in the system. A correctly set P parameter will casue the system response to oscillate around the set point. 

The derivative response will dampen the initial response but optimally only for a given frequency response, which will however change with changing speed, hence choosing a set of paramters always implies a trade off between stability and performance of the controller.
Giving up on response time, that is on how tight a turn the system will handle without much overshooting, will get a more stable trajecotry, but will also limit the maximum speed the controller is able to handle without becoming unbounded.


### Expected Performance on third party computers
Given the frequency response depends among others on the processing power of the computer system, it is well possible that the performance of this code, on  the test party system, might well be better or worse than displayed in the video below.
The system on which this code was tested is close to ancient ... so it's likley that on a third party testing this code might perform better, meaning altering the max_speed with a higher value parameter will get a stable vehicle control response at faster speed than displayed here.

### Video and Images

**Vides**
Download the videoscreen capture of an entire loop around the track in ziped form, gunzip and play.
This is the outcome using following paramters 

```
Kp  0.061751
Ki  0.004
Kd  -0.662543
Max_Speed 35
```
These are set by default, so just type 
``./pid ``

[PID Loop Video Download from here](https://github.com/QuantumCoherence/CarND-PID-Control-Project/blob/master/vokoscreen-2018-06-01_21-37-30.mkv.gz)

**Images**
The following images show two outoput screen captures of the Tweedle state machine process (see under Parameters Tuning and Coding Notes for details).
The word "Reset" at the beginning of the line indicates that the car left the paved road and so the error estiamtion process was stopped, the simulator reset and the next step of the Tweedle algorhitmn processed.
The word "Loop" instead indicates that the whole track was completed.
Each time a new lowest error was found, the best_error was updated and the next step of the Tweedle algorithm processed. In the end the progress is visible, even though very slow, given one loop requires aproximatley 600 simulation steps.

The images below show just a subset of the whole process.




![Tweedle Process Output sample 1](https://github.com/QuantumCoherence/CarND-PID-Control-Project/blob/master/Tweedle%20Output.jpg?raw=true)


![Tweedle Process Output sample 2](https://github.com/QuantumCoherence/CarND-PID-Control-Project/blob/master/Tweedle%20Output2.jpg?raw=true)

### Speed and Paramter Tuning
A fully functional Tweedle state machine was implemented that can perform the tweedle algorithm to estimate the optimal PID paramter values. The state machine automatically resets the simulator, whenever the vehicle goes off the paved road or simply hits the top of the curb on either side of the track.

The optional input "-tweedle" to the "pid" binary, will start the tuning process with the starting values 0.1, 0.1 , 0.1 resp. for P, I and D. However, this process is exceedingly long, because at the outstart most parameter values combianiton will lead to crashes and many resets are necessary to move forward with the estimation.

With a bit of heuristic, trial and error it is however possible to find a rough guess numerically quite close to a good starting point for all the parameters, such that the vehicle will remain on the track most of the time and the algorithm will find an optimal configuration in a much shorter number of iterations.

The manual process is quite simple:

```
1. Keep the speed low (see below)
2. Set I and D to zero and make a guess for P
2. If the car flies of the track, reduce P until the car tends to remain on the track on straight or slightly turning trajectory.
3. Add a negative value to D such that the vehicle can now stay on the track even for higher P values, that is for tighter turns, all while achieving some level of stability
4. Once the car remains on the track indefinitely, even if with visible instability, pass the guesstimated parameters as starting to point to the the "pid" binary using the -tweedle option on.
```


Using these paramters the Tweedle state machine can then relativley quickly find optimal P, I and D paramter values, in aproximately several 100s steps. Yes, it remains quite long even after a rough intial manual estiamtion, but comperatively much much shorter,than without using manually estimated initial values.
The images above show a sample of the process output while it was seeking the best optimal PID param setting.

The sample show a starting PID setting that was estimated manually:
P = 0.19
I = 0
D = -0.8

The final values (not shown in the images) that were then coded as default settings for the compliled binary, were:

```
Kp  0.061751
Ki  0.004
Kd  -0.662543
Max_Speed 35
```

*Estimation and control of the speed*

To control the speed and emulate a car driver behavior of breaking to slow down and avoid driving off the track as much as possible, a simple throttle algorithm was used that can be summarized as following:

throttle = (0.7-fabs(cte -prev_cte))*fabs(1-(speed/max_speed));

What this means is the follwing: 

1. when the ratio between speed and the predefined max_speed value, is close to one, the throttle is set to zero.
2. when the rate of increase of the cte error is bigger than 0.7, the car slows down.
3. When the car slows down, the cte error rate goes to zero, hence accleration is the applied normally.




### Coding and usage Notes
No throttle PID was implemented as a simple algorithm was sufficient to control the speed effectively.
A Tweedle State Machine was coded in the pid class, that fully implements the whole tweedle algorithm.
The pid binary accepts following optional parameters

``./pid -tweedle" `` 

triggeres the tweedle process with starting values 0.1, 0.1, 0.1, It stops when tol < 0.00001

``./pid -tweedle -sp <float> -si <float> -sd <float> -mv <float>`` 

start the pid controller with the specified p, i, d and max_speed -mv value

``./pid ``

triggers the pid controller with default parameters value as described above 


Comments in the code should be self explanatory . 

Please note despite the Tweedle algorithm being simple, the state machine implementing it instead isn't. As the algorithm gets spread out among states, reading and following the tweedle logic in the code, becomes counterintuitive. To make things more complicated,this particular state machine was implemented using transtional logic rather than topology only as state transition criteria. This makes the number of used states smaller at the expanse of code readability.  
The reason a state machine was used is because of the need to transfer control between the tweedle algorithm logic and the simulator. State transition logic is a natural representaton of such problems and has the advantage of retaining the latest state even when the connection with the simulator was to be lost half way during the parameters estimation process.   
Being able to restart the process without problems is a signficant advantge and therefore the state machine was implemented. 
As the tweedle state machine code is however outside the scope of the project, no detailed comments or documentation are made available.





