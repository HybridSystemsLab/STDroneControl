# ST Drone Simulation Code
We simulate each designed PID controller and then embed them in the MPC Switching Logic Framework. 

## Requirements
- MATLAB 2017 or above

## File Descriptions
- **init_state.m**: Helper function to initialize the states of the quadrotor.
- **drone_params.m**: Helper function that return a struct containing the parameters of the quadrotor.
- **DroneModel.m**: Helper file to evolve the states of the quadrotor. (Contains the forward euler descretized model of the quadrotor)
- **att_error_init.m, pos_error_init.m**: These files are used to initialize the PID controller errors. One is for attitude controller and the other is for position controller.
- **attitude_controller.m**: Attitude PID controller with high rise time.
- **attitudeController2.m**: Attitude PID controller with no overshoot.
- **inner_sim.m**: Main file used to test the attitude controllers and measure the performace of the those controllers.
- **position_controller.m**: Position PID controller with high rise time.
- **position_controller2.m**: Position PID controller with no overshoot.
- **outer_sim.m**: Main file used to test the position controllers and measure the performace of the those controllers.
- **generate_traj.m**: Helper file to geneerate a circular or a 8-figure traejctory.
- **main.m**: Main file that combines the designed PID controllers to track a circular or a 8-figure traejctory.
- **ControlEvolution.m**: Helper function used to evolve the PID controller forward in time for a prediction horizon.
- **DroneClosedLoopModel.m**: Helper function that evolves the quadrotor states given the thrust and moments.
- **EvaluateCost.m**: Helper function to compute the cost of each controller for a prediction horizon.
- **main_CL.m**: Main file containing the MPC Swithcing Logic Framework that combines the designed PID controllers.  

## How to Run the Code
###### Testing attitude controller
- Open the attitude controller files: **attitude_controller.m**, **attitudeController2.m**, **inner_sim.m**.
- In **inner_sim.m**, change line 55 depending on which controller needs to be tested.
- Run the code.
- Three Figures showing the desired angle vs the actual angle will show up. If the tracking requirement is not met, fine tune the gains and repeat the process.

###### Testing position controller
- Open the position controller files: **position_controller.m**, **position_controller2.m**, **outer_sim.m**.
- In **outer_sim.m**, change line 60 depending on which controller needs to be tested.
- Run the code.
- One Figures containing six subplots should show up. If the tracking requirement is not met, fine tune the gains and repeat the process.

###### Testing trajectory tracking
- Open the **main.m** file.
- In **main.m**, change line 55 depending on which position controller needs to be tested, and change line 63 depending on which attitude controller needs to be tested.
- Run the code.
- Two Figures should show up. The first is a 3D plot showing the tracking of the trajectory adn second is a subplot showing the errors in X, Y, and Z.

###### Running MPC Swithcing Logic Framework
- Open the **main_CL.m** file.
- In **main_CL.m**, lines 48 is the swithcing hysteresis parameter, line 49-50 are the parameters for the MPC framework, namely the PRediction horizon and Control Horizon.
- You can alter the cost weights by opening the **EvaluateCost.m** file.
- Run the code.
- Three Figures should show up. The first is a 3D plot showing the tracking of the trajectory and second is a subplot showing the errors in X, Y, and Z. The third would be a plot of the swithcing variable q.
