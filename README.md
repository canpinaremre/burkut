Team Kusbegi

Burkut FlightTask 

* Steps to follow:

1- Clone PX4/Firmware (please read note at the bottom)

2- "mc_pos_control_main.cpp" and "mc_pos_control_params.c" to "src/modules/mc_pos_control/"

3- "flight_tasks" folder to "src/lib/"

4- cd Firmware

5- make px4_sitl jmavsim

6- Open QGroundControl and take off

7- Change parameter "MPC_AUTO_MODE" to "2".(Advanced settings / Manuel entry)

8- After mission reset to default.

Upload to Pixhawk:

make px4_fmu-v5_default upload

(px4_fmu-v5_default should change for each pixhawk model.Use Pixhawk datasheet)

* Usage:

yaw speed controlled with NAV_LOITER_RAD which is unused by MC (FW only).Default yaw speed is 300.0

For faster speeds (NOT RECOMENDED!) yaw rate must be increased with parameter MC_YAWRATE_MAX

After these settings takeoff basicly to wanted altitude and change parameter MPC_AUTO_MODE to 2

When mission is done parameter will change to default value automaticly.

*** NOTE: After last update MPC_AUTO_MODE cannot be used.Use an older version of PX4 which contains this parameter.
