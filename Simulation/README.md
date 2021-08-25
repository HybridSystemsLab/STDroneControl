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
- **main_CL**: Main file containing the MPC Swithcing Logic Framework that combines the designed PID controllers.  

## How to Run the Code
The project is divided into two parts:
- Simulation (Look at the README in Simulation folder)
- Experiment (Look at the README in Experiment folder)

