%% ============================================================ %%
%                     Simple hovering algorithm 
%% ============================================================ %%


%% Close everything
close all;
clear all;
clc;

%% Connect to Motive
dllPath = fullfile('c:','Users','habhakta','Desktop','HSL','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP);

%% Connect to the Drone
b = ble("C0285B324333"); % Make sure to get the the Drone's MAC address before running this code

char = b.Characteristics; % Get the characteristics of the Drone


%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.
Drone_ID = 1;

%% Store the reference to the Charactersitic resposible for Writing Joystick data:
        % To send Joydata you need to send an array of 7 elements:
        % First element:- No idea what this does.
        % Second element:- rudder value (gRUD) 128 is like sending 0 (YAW)
        % Third element:- thrust value (gTHR) (THRUST)
        % Fourth element:- AIL value (gAIL) 128 is like sending 0  (ROLL)
        % Fifth element:- ELE value (gELE) 128 is like sending 0  (PITCH)
        % Sixth element:- SEEKBAR value: mainly from android app just send 0.
        % Seventh element:- ARMING and CALIB: 0x04, 0x05, 0x01(For calib)
joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B")

%% Next calibrate the drone
%calibrate the drone meaning reset the snesor data
write(joy_c, [0, 0, 0, 0, 0, 0, 2], 'uint8', "WithoutResponse")
%wait for 2 seconds
java.lang.Thread.sleep(2*1000);

write(joy_c, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");

%% Set up data collection vectors
ITERATIONS = 1050;
WARMUP = 100;

Drone_data = zeros(ITERATIONS + 2, 7);  % drone position data
% error_data = []; % position errors
sent_data = zeros(ITERATIONS + 1, 3); % data sent to drone
cont_actual_data = zeros(ITERATIONS + 1, 3);

p_x = zeros(ITERATIONS + 1, 1);
p_y = zeros(ITERATIONS + 1, 1);
p_z = zeros(ITERATIONS + 1, 1);

p_x_d = zeros(ITERATIONS, 1);
p_y_d = zeros(ITERATIONS, 1);

v_x_d = zeros(ITERATIONS, 1);
v_y_d = zeros(ITERATIONS, 1);

f_v_x = zeros(ITERATIONS + 1, 1);
f_v_y = zeros(ITERATIONS + 1, 1);
f_v_z = zeros(ITERATIONS + 1, 1);

vz_ref = zeros(ITERATIONS + 1, 1);
zz_ref = zeros(ITERATIONS + 1, 1);

PHI = zeros(ITERATIONS + 1, 1);
THETA = zeros(ITERATIONS + 1, 1);

%% Frequencies
OUT_FREQ = 60;
CUT_OFF_FREQ_POS = 10;
CUT_OFF_FREQ_VEL = 10;

%% Inititalize the PID controllers
X_pid = Xpid_error_init(1, OUT_FREQ, CUT_OFF_FREQ_POS);
Y_pid = Ypid_error_init(1);
Z_pid = Zpid_error_init(1);

%% Mass of the drone
m = 86.55/1000; % 86.55g
MAX_ANGLE = 30; % 30 deg

%% Get the Drone data
[DronePos] = GetDronePosition(theClient, Drone_ID);
Drone_data(1, :) = DronePos;

%% SET POINT TO TRACK
x_ref = 0.0;
y_ref = 0.0;
z_ref = 0.0;
psi = 0;
phi_d = 0;
theta_d = 0;

%% Initialize lowpass filter
[DronePos] = GetDronePosition(theClient, Drone_ID);
lpfData_x = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(2));
lpfData_y = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(3));
lpfData_z = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(4));

lpfData_vx = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vy = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vz = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

prev_x =  DronePos(2);
prev_y =  DronePos(3);
prev_z =  DronePos(4);
landing = 0;
%% Warm up Filter
for i = 1:100
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    %Drone_data = [Drone_data; DronePos];
    
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, x_f - prev_x);
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, y_f - prev_y);
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, z_f - prev_z);
    prev_z = z_f;
    
end

%% Keep track of previous position for computing velocity.
% prev_xx = DronePos(2);
% prev_yy = DronePos(3);
% prev_zz = DronePos(4);

k = 1;
p_z_ref = z_ref; 
while(k <= ITERATIONS)
%     k_bar = k - 1;
%     p_x_d(k) = 0.5*cos(0.01*k_bar);
%     p_y_d(k) = 0.5*sin(0.01*k_bar);
%     
%     v_x_d(k) = -0.01*0.5*sin(0.01*k_bar);
%     v_y_d(k) = 0.01*0.5*cos(0.01*k_bar);
    
    if(k <= 800)
        z_ref = z_ref + 1/OUT_FREQ *0.25;
    else
        landing = 1;
        z_ref = z_ref - 1/OUT_FREQ *0.25;
    end
    
    zz_ref(k) = z_ref;
    v_z_ref = (z_ref - p_z_ref)/(1/OUT_FREQ);
    vz_ref(k) = v_z_ref;
    p_z_ref = z_ref;
    
    if z_ref >= 1.0
        z_ref = 1.0;
    end
    
    if vz_ref(k) > 1.0
        vz_ref(k) = 1.0;
    elseif vz_ref(k) < -1.0
        vz_ref(k) = -1.0;   
    end
    
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    Drone_data(k+1, :) = DronePos;
    
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    p_x(k) = x_f;
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, (x_f - prev_x)/(1/120));
    f_v_x(k) = vx_f;
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    p_y(k) = y_f;
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, (y_f - prev_y)/(1/120));
    f_v_y(k) = vy_f;
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    p_z(k) = z_f;
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, (z_f - prev_z)/(1/120));
    f_v_z(k) = vz_f;
    prev_z = z_f;
    
    
    
    % Call X and Y Controller   
    [ddot_x_d, out_x, X_pid] = XCont(X_pid, x_ref, x_f, vx_f, 0.0, 2*1/OUT_FREQ);
    [ddot_y_d, out_y, Y_pid] = YCont(Y_pid, y_ref, y_f, vy_f, 0.0, 2*1/OUT_FREQ);

    % Call the Z Controller and retrive the thrust value
    [T, out_z, Z_pid] = Zcont(Z_pid, z_ref, z_f, vz_f, vz_ref(k), 1/OUT_FREQ);
    
    % Convert desired accelerations to desired angles
    z_T = 0.0043*double(T) + 0.001;

    if z_f < 0.95 && landing == 0
        phi_d = 0;
        theta_d = 0;
        X_pid.x_cumm_error = 0.0;
        Y_pid.y_cumm_error = 0.0;
    else
        phi_d = m/z_T * (ddot_x_d*cos(psi) + ddot_y_d*sin(psi))* 180/pi;
        theta_d = -m/z_T * (-ddot_y_d*cos(psi) + ddot_x_d*sin(psi)) * 180/pi;
    end
    
    % Cap the angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);
    
    PHI(k) = phi_d;
    THETA(k) = theta_d;
    
    % Convert the angles to 0 - 255
    slope_m = 255.0/(MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m *(phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m *(theta_d + MAX_ANGLE));
    
    % Colelct the data being sent
    sent_data(k, :) = [T, comm_phi_d, comm_theta_d];
    cont_actual_data(k, :) = [out_x, out_y, out_z];
    
    %Send the command to the Drone
    write(joy_c, [0, 128, T, comm_phi_d, comm_theta_d, 0, 5], 'uint8', "WithoutResponse")
    java.lang.Thread.sleep(1/OUT_FREQ*1000);
    k = k+1;
end

%% QUIT
write(joy_c, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");

% Finally, Close the Motive Client
theClient.Uninitialize();

disp('Done')
%end


% figure()
% plot(p_x, 'color', 'red')
% hold on
% plot(PHI, 'color', 'green')
% grid on
% 
% figure()
% plot(p_y, 'color', 'red')
% hold on
% plot(THETA, 'color', 'green')
% grid on
% 
% figure()
% plot(p_x(1), p_y(1), '*')
% hold on
% plot(p_x, p_y)
% axis([-1 1 -1 1])
% grid on




