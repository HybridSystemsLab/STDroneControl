clear all;
clc;
close all;

%------------- START OF MAIN -------------
%% System Constants for controller rates
display('OUTER LOOP SIMULATION')

ATTITUTDE_LOOP_RATE = 800; %Hz, use rates multiple of 10
POSITION_LOOP_RATE = 120;

dt = 1/ATTITUTDE_LOOP_RATE;
dt_pos = 1/POSITION_LOOP_RATE;

TT = 10; % total simulation time

time = 0:dt:TT; % time vector with step size dt to T
sim_length = length(time);

%% System parameters
params = drone_params();

%% Initialize the state and plotting data
state = init_state();

pos_err = pos_error_init(1);
att_err1 = att_error_init(1);
att_err2 = att_error_init(1);

T_plot = zeros(1,sim_length-2);

x_plot = zeros(1,sim_length-2);
y_plot = zeros(1,sim_length-2);
z_plot = zeros(1,sim_length-2);

theta_plot = zeros(1,sim_length-2);
phi_plot = zeros(1,sim_length-2);
psi_plot = zeros(1,sim_length-2);

des_x_plot = zeros(1,sim_length-2);
des_y_plot = zeros(1,sim_length-2);
des_z_plot = zeros(1,sim_length-2);

des_theta_plot = zeros(1,sim_length-2);
des_phi_plot = zeros(1,sim_length-2);
des_psi_plot = zeros(1,sim_length-2);

des_pos = [0.5; 0.5; 1; 0];

sine_wave = 0.25*sin(time);
square_wave = 0.25*square(time);

% desiredAngles.theta_d = 0;
% desiredAngles.phi_d = 0;
% desiredAngles.psi_d = 0;

%% MAIN LOOP
for i = 1:TT*ATTITUTDE_LOOP_RATE - 1    
    if (mod(i,ceil(ATTITUTDE_LOOP_RATE/POSITION_LOOP_RATE) ) == 0 || i == 1)
        [T, theta_d, phi_d, new_pos_err] = position_controller(pos_err, state, des_pos, dt_pos, params);
        pos_err = new_pos_err;
%         [T, theta_d, phi_d] = position_controller2(state, des_pos, dt_pos, params, 0);
    end

    desiredAngles.theta_d = (theta_d);
    desiredAngles.phi_d = (phi_d);
    desiredAngles.psi_d = 0;
    
%     [M, at2_er] = attitudeController2(att_err2, state, desiredAngles, dt);
    [M, at2_er] = attitude_controller(att_err2, state, desiredAngles, dt);
    att_err2 = at2_er;

    input = [T; M];
    
    % Apply the input to the Drone
    state = DroneModel(state, input, dt, params);
    
    %previousState = state;
     
     %% Saving data for plotting purposes
     T_plot(i) = T;
     
     x_plot(i) = state.x;
     y_plot(i) = state.y;
     z_plot(i) = state.z;
     
     theta_plot(i) = state.theta;
     phi_plot(i) = state.phi;
     psi_plot(i) = state.psi;
     des_x_plot(i) = des_pos(1);
     des_y_plot(i) = des_pos(2);
     des_z_plot(i) = des_pos(3);
     
     des_theta_plot(i) = theta_d;
     des_phi_plot(i) = phi_d;
     des_psi_plot(i) = 0;
end

%% Plot for comparision between z and desired z %%%%%%%%%
figure();
subplot(6,1,1)
hold on;
plot(time(1:end-2), z_plot,'color','blue','linewidth',2);
plot(time(1:end-2), des_z_plot,'r-.','color','blue','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$z,z_{d}$','FontSize',16,'Interpreter','latex')
legend({'$z$','$z_d$'},'FontSize',16,'Interpreter','latex')
% title({'$altitude_{B}$ Controller'},'FontSize',30,'Interpreter','latex')
grid on;
% set(gca,'fontsize',30)

% %% Plot for comparision between x and desired x %%%%%%%%%
% figure();
subplot(6,1,2)
hold on;
plot(time(1:end-2), x_plot,'r','color','red','linewidth',2);
plot(time(1:end-2), des_x_plot,'r-.','color','red','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$x,x_{d}$','FontSize',16,'Interpreter','latex')
legend({'$x$','$x_d$'},'FontSize',16,'Interpreter','latex')
% title({'$X_{A}$ Controller'},'FontSize',30,'Interpreter','latex')
grid minor
% set(gca,'fontsize',30)

% %% Plot for comparision between y and desired y %%%%%%%%%
% figure();
subplot(6,1,3)
hold on;
plot(time(1:end-2), y_plot,'r','color','green','linewidth',2);
plot(time(1:end-2), des_y_plot,'r-.','color','green','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$y,y_{d}$','FontSize',16,'Interpreter','latex')
legend({'$y$','$y_d$'},'FontSize',16,'Interpreter','latex')
% title({'$Y_{A}$ Controller'},'FontSize',30,'Interpreter','latex')
grid minor
% set(gca,'fontsize',30)

%% JUST PLOTS OF THETA, PHI vs the Desired
subplot(6,1,4)
hold on;
plot(time(1:end-2), theta_plot*180/pi,'r','color','magenta','linewidth',2);
plot(time(1:end-2), des_theta_plot*180/pi,'r--','color','magenta','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\theta,\theta_{d}$','FontSize',16,'Interpreter','latex')
legend({'$\theta$','$\theta_d$'},'FontSize',16,'Interpreter','latex')
grid on;

subplot(6,1,5)
hold on;
plot(time(1:end-2), phi_plot*180/pi,'r','color','green','linewidth',2);
plot(time(1:end-2), des_phi_plot*180/pi,'r--','color','green','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\phi,\phi_{d}$','FontSize',16,'Interpreter','latex')
legend({'$\phi$','$\phi_d$'},'FontSize',16,'Interpreter','latex')
grid on;

subplot(6,1,6)
hold on;
plot(time(1:end-2), psi_plot*180/pi,'r','color','green','linewidth',2);
plot(time(1:end-2), des_psi_plot*180/pi,'r--','color','green','linewidth',2);
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$\psi,\psi_{d}$','FontSize',16,'Interpreter','latex')
legend({'$\psi$','$\psi_d$'},'FontSize',16,'Interpreter','latex')
grid on;

