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

%% Frequencies
OUT_FREQ = 60;
CUT_OFF_FREQ_POS = 10;
CUT_OFF_FREQ_VEL = 10;

%% Set up arrays for data collection
Drone_data = [];
p_x = [];
p_y = [];
p_z = [];

v_x = [];
v_y = [];
v_z = [];

f_v_x = [];
f_v_y = [];
f_v_z = [];

theta_x = [];
phi_x = [];
psi_x = [];

f_thd = [];
f_phd = [];
f_psd = [];
Drone_ID = 1;

%% Initialize lowpass filter
[DronePos] = GetDronePosition(theClient, Drone_ID);
lpfData_x = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(2));
lpfData_y = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(3));
lpfData_z = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(4));

lpfData_theta = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(5));
lpfData_phi = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(6));
lpfData_psi = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(7));

lpfData_vx = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vy = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vz = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

lpfData_thd = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_phd = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_psd = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

prev_x =  DronePos(2);
prev_y =  DronePos(3);
prev_z =  DronePos(4);

prev_theta = DronePos(5);
prev_phi =  DronePos(6);
prev_psi =  DronePos(7);
%% Warm up Filter
for i = 1:100
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    Drone_data = [Drone_data; DronePos];
    
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, x_f - prev_x);
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, y_f - prev_y);
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, z_f - prev_z);
    prev_z = z_f;
    
    [theta_f, lpfData_theta] = lpf_2(lpfData_theta, DronePos(5));
    [thd_f, lpfData_thd] = lpf_2(lpfData_thd, theta_f - prev_theta);
    prev_theta = theta_f;
    
    [phi_f, lpfData_phi] = lpf_2(lpfData_phi, DronePos(6));
    [phd_f, lpfData_phd] = lpf_2(lpfData_phd, phi_f - prev_phi);
    prev_phi = phi_f;
    
    [psi_f, lpfData_psi] = lpf_2(lpfData_psi, DronePos(7));
    [psd_f, lpfData_psd] = lpf_2(lpfData_psd, psi_f - prev_psi);
    prev_psi = psi_f;
end

prev_xx = DronePos(2);
prev_yy = DronePos(3);
prev_zz = DronePos(4);
%% Get the Drone data
counter = 0;
while(1)
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    Drone_data = [Drone_data; DronePos];
    
    % Filter the data coming in from the motive capture system.
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    p_x = [p_x; x_f];
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, (x_f - prev_x)/(1/120));
    v_x = [v_x; (DronePos(2) - prev_xx)/(1/120)];
    f_v_x = [f_v_x; vx_f];
    prev_xx = DronePos(2);
    prev_x = x_f;
    
    [theta_f, lpfData_theta] = lpf_2(lpfData_theta, DronePos(5));
    theta_x = [theta_x; theta_f];
    [thd_f, lpfData_thd] = lpf_2(lpfData_thd, (theta_f - prev_theta)/(1/120));
    f_thd = [f_thd; thd_f];
    prev_theta = theta_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    p_y = [p_y; y_f];
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, (y_f - prev_y)/(1/120));
    v_y = [v_y; (DronePos(3) - prev_yy)/(1/120)];
    f_v_y = [f_v_y; vy_f];
    prev_y = y_f;
    prev_yy = DronePos(3);
    
    
    [phi_f, lpfData_phi] = lpf_2(lpfData_phi, DronePos(6));
    phi_x = [phi_x; phi_f];
    [phd_f, lpfData_phd] = lpf_2(lpfData_phd, (phi_f - prev_phi)/(1/120));
    f_phd = [f_phd; phd_f];
    prev_phi = phi_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    p_z = [p_z; z_f];
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, (z_f - prev_z)/(1/120));
    v_z = [v_z; (DronePos(4) - prev_zz)/(1/120)];
    f_v_z = [f_v_z; vz_f];
    prev_z = z_f;
    prev_zz = DronePos(4);
    
    [psi_f, lpfData_psi] = lpf_2(lpfData_psi, DronePos(7));
    psi_x = [psi_x; psi_f];
    [psd_f, lpfData_psd] = lpf_2(lpfData_psd, (psi_f - prev_psi)/(1/120));
    f_psd = [f_psd; psd_f];
    prev_psi = psi_f;
    
    java.lang.Thread.sleep(1/OUT_FREQ*1000);
    % Live plot
    h_p = plot(Drone_data(1:end,5)*180/pi, 'color', 'red');
    pause(eps)
    delete(h_p);
end

figure(); 
plot(Drone_data(100:end,2), 'color', 'red')
hold on; 
plot(p_x, 'color', 'blue'); 

figure();
plot(v_x, 'color', 'red');
hold on
plot(f_v_x, 'color', 'blue')
% Finally, Close the Motive Client
theClient.Uninitialize();

disp('Done')
%end