# ST Drone Experiment Code
We use the Uniting Control strategy to hover the ST drone at some desired altitude.

## Requirements
- Bluetooth Recevier/Transmitter
- MATLAB 2019 or above
- ST Drone Kit
- Reflective markers
- Motion capture system SDK
- Motive

## File Descriptions
- **lpf_2_init.m**: Helper function to initialize the low pass filter and returnt he parameter as a struct.
- **lpf_2.m**: Helper fucntion to perform low pass filtering on sampled data.
- **GetDronePosition.m**: Helper function to get the position and orientation of the rigid body defined in Motive, software used to localize the rigid body using the Optitrack motion capture system data.
- **DroneDataCollector.m**: Main file to validate the working of collecting data of the rigid body and filtering it using the low pass filter.
- **Xpid_error_init.m, Ypid_error_init.m, Zpid_error_init.m**: These files are used to initialize the X, Y, and Z PID controller errors respectively. 
- **XCont.m, YCont.m, Zcont.m**: Experimentally designed X, Y, and Z PID controller respectively.
- **main.m**: Main file to design the controllers experimentally and hover the ST Drone.
- **VXpid_error_init.m, VYpid_error_init.m, VZpid_error_init.m**: These files are used to initialize the X, Y, and Z PIV controller errors respectively. 
- **position_controller.m, VZcont.m**: Experimentally designed X, Y, and Z PIV controller respectively.
- **Zpos_main.m**: Main file to design the PIV controllers experimentally and to have the ST Drone hover at certain altitude.
- **Zcont_2.m**: Experimentally designed altitude controller for no overshot and lower rise time compared to **Zcont.m**.
- **pos_main.m**: Main file implementing the Uniting Control strategy. 

## How to Run the Code
###### Steps for ST Drone
- Place reflective markers on the ST Drone.
- Connect the battery to the ST Drone.
- Place the ST Drone on a flat surface and press the reset button.

###### Steps for Motion Capture System
- Open Motive
- Calibrate the Motion Capture System.
- Create a rigid body in Motive.
- Open the broadcast pane, check broadcast data, and then select _local loop back_.

###### Running the Experiment
- Open MATLAB 2019 or above
- In **main.m** (line 12), **Zpos_main.m** (line 7), and **pos_main.m** (line 7), the script connects to the Motive client using the NatNet SDK which can be found in the utils folder. Make sure to update the path to where this SDK is lcoated on your computer.
- In the command line type blelist and press enter. Copy the MAC address of DRN1110 device.
- Paste the MAC address where the script tries to connect to the ST Drone. For example `b = ble("C0285B324333");`, replace `C0285B324333` with the MAC address of the ST Drone.
- Double check that Motive is configured to send data to MATLAB.
- Run the code.
