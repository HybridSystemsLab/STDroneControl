clear all;
clc;
close all;

%------------- START OF MAIN -------------
%% System Constants for controller rates
display('INNER AND OUTER PID SIMULATION TO TRACK CIRCULAR TRAJECTORY')
%%% InnerLoopRate should be the fastest or at-least equal to outerloop.

ATTITUTDE_LOOP_RATE = 500; %Hz, use rates multiple of 10
POSITION_LOOP_RATE = 50; %Hz , use rates multiple of 10
% HEIGHT_CONTROLLER_RATE = 50; %Hz , use rates multiple of 10

dt = 1/ATTITUTDE_LOOP_RATE;
dt_pos = 1/POSITION_LOOP_RATE;

TT = 80; % total simulation time

time = 0:dt:TT; % time vector with step size dt to T
sim_length = length(time);
%% System parameters
params = drone_params();
pos_err = pos_error_init(1);
att_err = att_error_init(1);
%% Trajectory
des = generate_traj(1, TT, dt);

%% Initialize the state and plotting data
state = init_state();

roll_plot = zeros(1,sim_length-2);
pitch_plot = zeros(1,sim_length-2);
yaw_plot = zeros(1,sim_length-2);

des_roll_plot = zeros(1,sim_length-2);
des_pitch_plot = zeros(1,sim_length-2);
des_yaw_plot = zeros(1,sim_length-2);

z_plot = zeros(1,sim_length-2);
x_plot = zeros(1,sim_length-2);
y_plot = zeros(1,sim_length-2);
x_error_plot = zeros(1,sim_length-2);
y_error_plot = zeros(1,sim_length-2);

DesiredAnglesOld = [0;0;0];
Omega_d = [0;0;0];
Omega_d_old = Omega_d;

%% MAIN LOOP
for i = 1:TT*ATTITUTDE_LOOP_RATE - 1
    % Run position controller at a rate slower than that off the attitude
    % controller.
    if (mod(i,ceil(ATTITUTDE_LOOP_RATE/POSITION_LOOP_RATE) ) == 0 || i == 1)
        desired_pos = [des.pos(i,:), des.yaw(i)];
        [T, theta_d, phi_d, p_er] = position_controller2(pos_err, state, desired_pos, dt_pos, params);
        pos_err = p_er;
    end
    
    desiredAngles.theta_d = theta_d;
    desiredAngles.phi_d = phi_d;
    desiredAngles.psi_d = des.yaw(i);
    
    [M, att_er] = attitude_controller(att_err, state, desiredAngles, dt);
    att_err = att_er;
    
    input = [T; M];
    state = DroneModel(state, input, dt, params);
    
    previousState = state;
     
     %% Saving data for plotting purposes
     x_plot(i) = state.x;
     y_plot(i) = state.y; 
     z_plot(i) = state.z;
     
     roll_plot(i) = state.phi;
     pitch_plot(i) = state.theta;
     yaw_plot(i) = state.psi;
     
     x_error_plot(i) = state.x - des.pos(i,1);
     y_error_plot(i) = state.y - des.pos(i,2);
     
     des_roll_plot(i) = desiredAngles.phi_d;
     des_pitch_plot(i) = desiredAngles.theta_d;
     des_yaw_plot(i) = desiredAngles.psi_d;
     
end


%% --- PLOTTING ERRRORS ---
figure(1);
hold on;
grid minor;
title({'$position_{B}$ and $altitude_{A}$ Controller'},'FontSize',30,'Interpreter','latex')
plot3(des.pos(:,1),des.pos(:,2),des.pos(:,3), 'r.','color','green','linewidth',2)
plot3(x_plot,y_plot,z_plot, 'r.','color','blue','linewidth',2);
xlabel('$X(m)$','FontSize',16,'Interpreter','latex')
ylabel('$Y(m)$','FontSize',16,'Interpreter','latex')
zlabel('$Z(m)$','FontSize',16,'Interpreter','latex')
set(gca,'fontsize',20)

X_des = des.pos(:,1);
Y_des = des.pos(:,2);
Z_des = des.pos(:,3);

X_des = X_des(1:end-2);
Y_des = Y_des(1:end-2);
Z_des = Z_des(1:end-2);

X_E = X_des - x_plot';
Y_E = Y_des - y_plot';
Z_E = Z_des - z_plot';

figure();
subplot(3,1,1)
title({'Tracking Errors'},'FontSize',25,'Interpreter','latex')
hold on;
grid on;
% plot(time(1:end), des.pos(:,1), 'r--','color','red','linewidth',2)
plot(time(1:end-2), X_E, 'r.','color','red','linewidth',2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$X_{error}(m)$','FontSize',16,'Interpreter','latex')
grid minor;
xlim([0 30])
set(gca,'fontsize',20)

subplot(3,1,2)
hold on;
grid minor;
% plot(time(1:end), des.pos(:,2), 'r--','color','green','linewidth',2)
% plot(time(1:end-2), y_plot, 'r.','color','red','linewidth',2)
plot(time(1:end-2), Y_E, 'r.','color','black','linewidth',2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$Y_{error}(m)$','FontSize',16,'Interpreter','latex')
xlim([0 30])
set(gca,'fontsize',20)

subplot(3,1,3)
hold on;
grid minor;
% plot(time(1:end), des.pos(:,3), 'r--','color','blue','linewidth',2)
plot(time(1:end-2), Z_E, 'r.','color','blue','linewidth',2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$Z_{error} (m)$','FontSize',16,'Interpreter','latex')
xlim([0 30])
set(gca,'fontsize',20)
% %% Plotting x error
% figure(2);
% hold on;
% plot(time(1:end-2), x_error_plot,'r','color','blue','linewidth',2);
% xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
% ylabel('$e_x$','FontSize',16,'Interpreter','latex')
% legend({'$ error $'},'FontSize',16,'Interpreter','latex')
% grid on;
% 
% %% Plot for comparision between roll angle and desired roll angle %%%%%%%%%
% figure();
% hold on;
% plot(time(1:end-2), roll_plot*(180/pi),'r','color','blue','linewidth',2);
% plot(time(1:end-2), des_roll_plot*(180/pi),'r--','color','blue','linewidth',2);
% xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
% ylabel('$\phi,\phi_{d}$','FontSize',16,'Interpreter','latex')
% legend({'$\phi$','$\phi_d$'},'FontSize',16,'Interpreter','latex')
% grid on;
% 
% %% Plot for comparision between pitch angle and desired pitch angle %%%%%%%%%
% figure();
% hold on;
% plot(time(1:end-2), pitch_plot*(180/pi),'r','color','green','linewidth',2);
% plot(time(1:end-2), des_pitch_plot*(180/pi),'r--','color','green','linewidth',2);
% xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
% ylabel('$\theta,\theta_{d}$','FontSize',16,'Interpreter','latex')
% legend({'$\theta$','$\theta_d$'},'FontSize',16,'Interpreter','latex')
% grid on;
% 
% %% Plot for comparision between yaw angle and desired pitch angle %%%%%%%%%
% figure();
% hold on;
% plot(time(1:end-2), yaw_plot*(180/pi),'r','color','red','linewidth',2);
% plot(time(1:end-2), des_yaw_plot*(180/pi),'r--','color','red','linewidth',2);
% xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
% ylabel('$\psi,\psi_{d}$','FontSize',16,'Interpreter','latex')
% legend({'$\psi$','$\psi_d$'},'FontSize',16,'Interpreter','latex')
% grid on;
