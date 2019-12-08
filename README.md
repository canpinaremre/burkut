Team Kusbegi

Burkut FlightTask 

Steps to follow:

1- Clone PX4/Firmware

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
