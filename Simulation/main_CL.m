clear all;
clc;
close all;

%------------- START OF MAIN -------------
%% System Constants for controller rates
display('INNER PID PLUS PLANT CLOSED LOOP SIMULATION')
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

pos_err1 = pos_error_init(1);
pos_err2 = pos_error_init(1);

att_err1 = att_error_init(1);
att_err2 = att_error_init(1);

%% Trajectory
des = generate_traj(1, TT, dt);

%% Initialize the state and plotting data
pre_state = init_state();
actual_state = init_state();
state_1 = init_state();
state_2 = init_state();

z_plot = zeros(1,sim_length-2);
x_plot = zeros(1,sim_length-2);
y_plot = zeros(1,sim_length-2);

q = zeros(1,sim_length-2);
cost_1 = zeros(1,sim_length-2);
cost_2 = zeros(1,sim_length-2);

%% PARAMETER MU
mu = 1.02; %101;
Prediction_Horizon = 0.1; %in seconds
Control_Horizon = 5;


%% MAIN LOOP
% for i = 1:TT*ATTITUTDE_LOOP_RATE - 1
i = 1;
switcher = 0;
while i <= TT*ATTITUTDE_LOOP_RATE - 1
    state_1 = actual_state;
    state_2 = actual_state;
        
    [r_p1, p_p1, ya_p1, des_r_p1, des_p_p1, des_y_p1, z_p1, x_p1, y_p1, x_er_p1, y_er_p1, z_er_p1, state_v1, u_0, p1_err, att1_err] = ControlEvolution(Prediction_Horizon, params, des, state_1, pos_err1, att_err1, 0, i-1);
  
    [r_p2, p_p2, ya_p2, des_r_p2, des_p_p2, des_y_p2, z_p2, x_p2, y_p2, x_er_p2, y_er_p2, z_er_p2, state_v2, u_1, p2_err, att2_err] = ControlEvolution(Prediction_Horizon, params, des, state_2, pos_err2, att_err2, 1, i-1);
    


    % evaluate the cost
    cont0_cost = EvaluateCost(x_er_p1, y_er_p1, z_er_p1, u_0);
    cont1_cost = EvaluateCost(x_er_p2, y_er_p2, z_er_p2, u_1);

    cost_1(i) = cont0_cost;
    cost_2(i) = cont1_cost;
    
%     apply_control = [0, 0, 0, 0];
    % pick the controller with the least cost: Do this by paper 19 logic!
    for j=1:Control_Horizon
        if (cont1_cost > mu*cont0_cost) %switcher > 0 && switcher < 3
            apply_control =  u_0(j,:);
            
            % Evolve the original system for how long? try 1 step. Basically we are
            % taking the first input computed. Now doing it for 10 steps
            actual_state = DroneModel(actual_state, apply_control, dt, params);
            pos_err1 = p1_err(j);
            att_err1 = att1_err(j);
            pos_err2 = p1_err(j);
            att_err2 = att1_err(j);
            
            x_plot(i+j-1) = actual_state.x;
            y_plot(i+j-1) = actual_state.y; 
            z_plot(i+j-1) = actual_state.z;
            
            q(i+j-1) = 0;
            
        else
            apply_control =  u_1(j, :);
            
            actual_state = DroneModel(actual_state, apply_control, dt, params);
            pos_err1 = p2_err(j);
            att_err1 = att2_err(j);
            pos_err2 = p2_err(j);
            att_err2 = att2_err(j);
            
            
            
            
            x_plot(i+j) = actual_state.x;
            y_plot(i+j) = actual_state.y; 
            z_plot(i+j) = actual_state.z;
            
            q(i+j) = 1;
        end
    end
    
    i = i+j;
    switcher = switcher + 1;
    if  switcher > 5
        switcher = 0;
    end
    

end    

figure(2);
hold on;
grid on;
plot3(des.pos(:,1),des.pos(:,2),des.pos(:,3), 'r.','color','green','linewidth',2)
plot3(x_plot(1:i),y_plot(1:i), z_plot(1:i), 'r.','color','red','linewidth',2);
xlabel('$X(m)$','FontSize',16,'Interpreter','latex')
ylabel('$Y(m)$','FontSize',16,'Interpreter','latex')
zlabel('$Z(m)$','FontSize',16,'Interpreter','latex')

figure();
hold on;
grid on;
plot3(des.pos(:,1),des.pos(:,2),des.pos(:,3), 'r.','color','green','linewidth',2)
plot3(x_plot(1:i),y_plot(1:i), z_plot(1:i), 'r.','color','red','linewidth',2);
xlabel('$X(m)$','FontSize',16,'Interpreter','latex')
ylabel('$Y(m)$','FontSize',16,'Interpreter','latex')
zlabel('$Z(m)$','FontSize',16,'Interpreter','latex')


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
hold on;
grid on;
% plot(time(1:end), des.pos(:,1), 'r--','color','red','linewidth',2)
plot(time(1:end-2), X_E, 'r.','color','red','linewidth',2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$X_{error}(m)$','FontSize',16,'Interpreter','latex')

subplot(3,1,2)
hold on;
grid on;
% plot(time(1:end), des.pos(:,2), 'r--','color','green','linewidth',2)
% plot(time(1:end-2), y_plot, 'r.','color','red','linewidth',2)
plot(time(1:end-2), Y_E, 'r.','color','black','linewidth',2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$Y_{error}(m)$','FontSize',16,'Interpreter','latex')

subplot(3,1,3)
hold on;
grid on;
% plot(time(1:end), des.pos(:,3), 'r--','color','blue','linewidth',2)
plot(time(1:end-2), Z_E, 'r.','color','blue','linewidth',2)
xlabel('$t(sec)$','FontSize',16,'Interpreter','latex')
ylabel('$Z_{error} (m)$','FontSize',16,'Interpreter','latex')


figure()
plot(q)
%% Plot the DATA
% plotData(des, time(1:499), x_p1, y_p1, z_p1, x_er_p1, r_p1, p_p1, ya_p1, des_r_p1, des_p_p1, des_y_p1)
% plotData(des, time(1:499), x_p2, y_p2, z_p2, x_er_p2, r_p2, p_p2, ya_p2, des_r_p2, des_p_p2, des_y_p2)