# README File for Sensors and Control Group Assignment

Students in group and associated contribution percentages to the code development:
Liam Thurston (98102534) - 40%
Harrison Wittke (12545012) - 40%
Rhys Edwards (12618272) - 20%

Project Brief:
The objective of this assignment was to utilise sensor information captured through the RGB-D camera mounted on the follower turtlebot to identify, locate and follow the lead turtlebot in the Gazebo simulation environment. It was the goal of the project team that the lead robot would have a unique identifier tag mounted to it to assist in the object identification and pose estimation tasks. The follower robot was then programmed to follow the lead robot at a distance of 0.5m.


Submission Folder Contents:
/src directory - containing all required ROS packages in appropriate format (DO NOT ALTER THE FOLDER STRUCTURE OF THIS SRC FOLDER)
/matlab_code directory - containing all MATLAB files used to control the follower robot

Steps to correctly set-up and initialise the system to complete the project task:
1. Delete existing /build and /src directories from ~/catkin_ws/ directory on local machine (if you do not wish to delete your existing /src directory, copy all the sub-folders contained in the /src directory in the submission folder. It is ESSENTIAL that the folders are copied into the existing /src directory as underlying URDF and model files were altered in the turtlebot3 folders to include the unique identifier tag on the leader robot).
2. Copy the /src folder and all sub-folders into the ~/catkin_ws/ directory on the local machine
3. Run 'catkin_make'

YOU WILL NEED TO INSTALL THE CUSTOM ROS MESSAGES LIBRARY IN MATLAB AS THE /ar_pose_marker TOPIC IS NOT RECOGNISED AS A STANDARD MESSAGE IN MATLAB

4. Open MATLAB.
5. Type 'rosAddons' into the MATLAB Command Prompt
6. Install the 'Robotics System Toolbox Interface for ROS Custom Messages' library
7. Once installed type 'folderpath = '~/catkin_ws/ar_track_alvar/' into the MATLAB Command Prompt
8. Type 'rosgenmsg(folderpath)' into the MATLAB command prompt
9. Follow the three steps presented in the Command Prompt window:
    a. Edit javaclasspath.txt, add the following listed file locations as new lines, and save the file.
    b. Add the custom message folder to the MATLAB path by executing: 
        addpath('FILEPATH DISPLAYED')
        savepath
    c. Restart MATLAB and verify that you can use the custom messages. Type "rosmsg list" and ensure that the output contains the generated custom message types. These should be as follows:
	i.   ar_track_alvar/AlvarMarker                                     
	ii.  ar_track_alvar/AlvarMarkers                                    
	iii. ar_track_alvar_msgs/AlvarMarker                                
	iv.  ar_track_alvar_msgs/AlvarMarkers   

HAVING SUCCESSFULLY INSTALLED THE CUSTOM ROS MESSAGES LIBRARY IN MATLAB YOU CAN NOW RUN THE SIMULATION

10. Open the 'main.m' script in MATLAB (DO NOT RUN IT YET).
11. Navigate back to the terminal and launch the simulation using 'roslaunch turtlebot3_gazebo two_turtlebot3.launch'
12. To control the lead Turtlebot launch the control package using 'roslaunch turtlebot3_teleop leadRobot'
13. Run the 'main.m' MATLAB script.
14. To stop the simulation type Ctrl+C into MATLAB and the running ROS nodes.


It is VERY important that the above steps are followed closely to ensure the correct operation of our project demonstration. It is also EXTREMELY important that the folders that have been submitted are used, as some of the underlying URDF files have been edited to ensure correct execution of our simulation.

A full list of the edited files created and/or modified by the project team is below:
'~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/two_turtlebot3.launch' - Required launch file for the successful execution of the simulation.
'~/catkin_ws/src/turtlebot3/turtlebot3_teleop/launch/leadRobot.launch' - used to launch the control of only the lead turtlebot.
'~/catkin_ws/src/ar_tag_demo/' - and ALL underlying sub-folders/files, used to launch the tracking functionality and store the tag bundle used in the simulation.
'./matlab_code/' - and ALL underlying files. MATLAB code used to perform the tracking of the leader turtlebot and control of the follower turtlebot as required in the project brief.


'~/catkin_ws/src/turtlebot3/turtlebot3_teleop/nodes/leadRobot' - node used to launch the control of the lead turtlebot.
'~/catkin_ws/src/turtlebot3/turtlebot3_description/meshes/arTag/' - and all underlying files, used to create the mesh of the AR Tag to display in the Gazebo simulation.
'~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf.xacro' - edited to attach the AR Tag to the rear of the lead turtlebot.
'~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi.gazebo.xacro' - edited to attach the AR Tag to the rear of the lead turtlebot.
'~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/meshes/ar_tag_0_to_3.dae' - used to generate the AR Tag in Gazebo and fix it to the rear of the lead turtlebot.
'~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi/meshes/Markers_0to3.png' - referenced by the dae and urdf files in generating the AR Tag in the Gazebo simulation.
'~/catkin_ws/src/ar_track_alvar' - and all underlying sub-folders/files, ROS package used to track the AR Tags attached to the lead turtlebot.


