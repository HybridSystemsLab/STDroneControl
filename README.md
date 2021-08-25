# ST Drone Control
Using the quarotor model derived in Chapter 3 of the paper, we design two sets of Proportional-Integral-Derivative (PID) controllers to control the altitude, position and attitude of the quadrotor. The first set of controllers are designed such that they have fast rise time while the second set of controllers are designed such that there is no overshoot. An MPC like framework is presented, in which we propagate the states of the quadrotor using the PID controllers and switch between them to attain better performance. Lastly, we design uniting control strategy and is demonstrated in an experiment using the ST Drone.

## Requirements
- Bluetooth Recevier/Transmitter
- MATLAB 2019 or above

## How to Run the Code
The project is divided into two parts:
- Simulation (Look at the README in Simulation folder)
- Experiment (Look at the README in Experiment folder)


