# MSBRC_GroupTask

HOW TO CLONE this project
1. Make sure, ROS Kinetic is installed
2. Create a new ROS workspace "MSBRDM_final_project"
3. Clone this directory into "MSBRDM_final_project/src"
4. Compile the workspace

Explanation:
 This directory includes all source data for the MSBRDM group project "Bartender's Little Helper" 
 (created in Winter Term 2019/20).

File structure:
 - libs: directory containing necessary TUM ICS libraries 
 - presentations: contains all presentations conducted during the project
 - robot_data: contains all information on the UR10 robot (manual, DH-convention)
               & the MATLAB files including the kinematics & dynamics computation
 - skin_data: contains information on the ICS robot skin
 - tum_ics_skin_tutorials: ROS package of a tutorial on the TUM ICS skin data
 - tum_ics_ur10_controller_tutorial: ROS package of a tutorial on the ur10 control, as implemented at TUM ICS
 - ur10_controller: ROS package of our own controller for "Bartender's Little Helper"
