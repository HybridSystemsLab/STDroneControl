clear all;
clc;
%close all;

%------------- START OF MAIN -------------
%% System Constants for controller rates
display('ATTITUDE LOOP SIMULATION')
%%% InnerLoopRate should be the fastest or at-least equal to outerloop.
ATTITUTDE_LOOP_RATE = 500; %Hz, use rates multiple of 10

dt = 1/ATTITUTDE_LOOP_RATE;

TT = 10; % total simulation time

time = 0:dt:TT; % time vector with step size dt to T
sim_length = length(time);

square_wave = 2.5*square(time);
sine_wave = 2.5*sin(time);
%% System parameters
params = drone_params();

%% Initialize the state and plotting data
state = init_state();
att_err1 = att_error_init(1);
att_err2 = att_error_init(1);

M_plot = zeros(sim_length-2, 3);

roll_plot = zeros(1,sim_length-2);
pitch_plot = zeros(1,sim_length-2);
yaw_plot = zeros(1,sim_length-2);

des_roll_plot = zeros(1,sim_length-2);
des_pitch_plot = zeros(1,sim_length-2);
des_yaw_plot = zeros(1,sim_length-2);

desiredAngles.theta_d = deg2rad(5); %deg2rad(square_wave(1));
desiredAngles.phi_d = deg2rad(5); %deg2rad(square_wave(i));
desiredAngles.psi_d = deg2rad(5); %deg2rad(square_wave(i));

%% MAIN LOOP
for i = 1:TT*ATTITUTDE_LOOP_RATE - 1    
    desiredAngles.theta_d = deg2rad(square_wave(i));
    desiredAngles.phi_d = deg2rad(square_wave(i));
    desiredAngles.psi_d = deg2rad(square_wave(i));
    
%     desiredAngles.theta_d = deg2rad(sine_wave(i));
%     desiredAngles.phi_d = deg2rad(sine_wave(i));
%     desiredAngles.psi_d = deg2rad(sine_wave(i));
    
    % Change the controller to find the response of it.
%     [M, at1_er]= attitude_controller(att_err1, state, desiredAngles, dt);
%       att_err1 = at1_er;
    [M, at2_er] = attitudeController2(att_err2, state, desiredAngles, dt);
    att_err2 = at2_er;
    input = [0; M];
    state = DroneModel(state, input, dt, params);
    
    previousState = state;
     
     %% Saving data for plotting purposes
     M_plot(i,:) = M';
     
     roll_plot(i) = state.phi;
     pitch_plot(i) = state.theta;
     yaw_plot(i) = state.psi;

     
     des_roll_plot(i) = desiredAngles.phi_d;
     des_pitch_plot(i) = desiredAngles.theta_d;
     des_yaw_plot(i) = desiredAngles.psi_d;
     
end

% Plot for comparision between roll angle and desired roll angle %%%%%%%%%
figure();
hold on;
plot(time(1:end-2), roll_plot*(180/pi),'r','color','blue','linewidth',2);
plot(time(1:end-2), des_roll_plot*(180/pi),'r--','color','blue','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\phi,\phi_{d}$','FontSize',16,'Interpreter','latex')
ylim([-5 5])
legend({'$\phi$','$\phi_d$'},'FontSize',16,'Interpreter','latex')
title({'$roll_{B}$ Controller'},'FontSize',30,'Interpreter','latex')
grid minor;
set(gca,'fontsize',30)

%% Plot for comparision between pitch angle and desired pitch angle %%%%%%%%%
figure();
hold on;
plot(time(1:end-2), pitch_plot*(180/pi),'r','color','green','linewidth',2);
plot(time(1:end-2), des_pitch_plot*(180/pi),'r--','color','green','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\theta,\theta_{d}$','FontSize',16,'Interpreter','latex')
ylim([-5 5])
legend({'$\theta$','$\theta_d$'},'FontSize',16,'Interpreter','latex')
title({'$pitch_{B}$ Controller'},'FontSize',30,'Interpreter','latex')
grid minor;
set(gca,'fontsize',30)

% Plot for comparision between yaw angle and desired pitch angle %%%%%%%%%
figure();
hold on;
plot(time(1:end-2), yaw_plot*(180/pi),'r','color','red','linewidth',2);
plot(time(1:end-2), des_yaw_plot*(180/pi),'r--','color','red','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\psi,\psi_{d}$','FontSize',16,'Interpreter','latex')
ylim([-5 5])
legend({'$\psi$','$\psi_d$'},'FontSize',16,'Interpreter','latex')
title({'$yaw_{B}$ Controller'},'FontSize',30,'Interpreter','latex')
grid minor;
set(gca,'fontsize',30)
