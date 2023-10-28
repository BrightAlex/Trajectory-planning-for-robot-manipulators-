%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory planning of robot manipulators
% Author: Yuejiang Zhu from RCLLab, CSE@SYSU
% Date: 2023/10/28
% email: zhuyj69@mail2.sysu.edu.cn
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% This scirpt generates and visulizes the the trajectories in both cartesian space and joint
% space. The output files are stored in results fold
close all;
clc;
clear;

robot_type = 8; % 8 supported robots in this project, can be added if necessary
shape_type = 1; % square:1, fish:2, rose:3, infinitylike:4
degree = -30;
alpha = degree/180*pi; % angle between square face and XZ plane in square shape
radius = 0.2; % amplitude for 4 shapes
n = 3; % number of petals for rose shape
interploration_type = 1; % Minimum-Jerk Trajectory:1, Trapezoidal Velocity Profile Trajectory:2,  Cubic and Quintic Polynomial Trajectories:3, B-Spline Trajectory:4
t_begin = 0; % start time of trajectory (s)
T = 5; % period of the trajectory (s)
t_end = t_begin + T; % end time of trjectory (s)
Ts_s = 0.1; % sampling period of waypoints to be used for inverse kinematics and interploration (s)
Ts_p = 0.001; % period of final trajectories (s)
Ts_ani = 0.02; % sampling period of the visulization

tvec_s = t_begin:Ts_s:t_end;
tvec_p = t_begin:Ts_p:t_end;

% available robots
% 'abbIrb120', 'abbIrb120T', 'abbIrb1600', 'abbYuMi', 'atlas', 'fanucLRMate200ib', 'fanucM16ib', 'frankaEmikaPanda',
% 'kinovaGen3', 'kinovaJacoJ2N6S200', 'kinovaJacoJ2N6S300', 'kinovaJacoJ2N7S300', 'kinovaJacoJ2S6S300',
% 'kinovaJacoJ2S7S300', 'kinovaJacoTwoArmExample', 'kinovaMicoM1N4S200', 'kinovaMicoM1N6S200', 'kinovaMicoM1N6S300',
% 'kukaIiwa7', 'kukaIiwa14', 'meca500r3', 'omronEcobra600', 'puma560', 'quanserQArm', 'rethinkBaxter', 'rethinkSawyer',
% 'robotisOP2', 'robotisOpenManipulator', 'universalUR10', 'universalUR10e', 'universalUR16e', 'universalUR3',
% 'universalUR3e', 'universalUR5', 'universalUR5e', 'valkyrie', 'yaskawaMotomanMH5', 'kinovaMovo', 'willowgaragePR2',
% 'amrPioneerLX', 'amrPioneer3AT', 'amrPioneer3DX', 'clearpathHusky', 'clearpathJackal', 'clearpathTurtleBot2',
% 'robotisTurtleBot3Burger', 'robotisTurtleBot3Waffle', 'robotisTurtleBot3WaffleForOpenManipulator',
% 'robotisTurtleBot3WafflePi', 'robotisTurtleBot3WafflePiForOpenManipulator', 'quanserQBot2e', 'quanserQCar',
% 'robotiq2F85'


robot_names = {"FrankaEmikaPanda", "rethinkSawyer", "universalUR5", "fanucM16i", "kukaIiwa7", "abbIrb1600", "kinovaGen3", "puma560"};
shape_names = {"square", "fish", "rose", "infinity"};
interpolation_names = {"Minimum-Jerk", "Trapezoidal Velocity", "Cubic and Quintic Polynomial", "B-Spline"};
end_link_names = {"panda_hand", "right_l6","ee_link","tool0","iiwa_link_ee","tool0", "EndEffector_Link", "link7"}; % to be added by user
save_name = robot_names{robot_type}+'_' + shape_names{shape_type} + '_' + interpolation_names{interploration_type};
path = ".\results\"+ save_name +"\";
robot = loadrobot(robot_names{robot_type},DataFormat='column'); 

dofs = {7, 8, 6, 6, 7, 6, 7, 6}; % the degrees of freedom,  to be added by user
dof = dofs{robot_type};
q0_panda = [0 0 0 -pi/2 0 pi/2 0 0 0]'; % initial joint position, which could make z axis of end link vertial downward the ground
q0_sawyer = [0 0 -pi/3 0 pi/3 0 pi/2 0]';
q0_ur5 = [ 0 -pi/2 pi/2 0 0 0 ]';
q0_fanucM16i = [0 0 0 0 0 0 ]';
q0_kuka = [0 0 -pi/2 -pi/2 0 pi/2 0]';
q0_abb = [ 0 0 0 0 0 0 ]';
q0_kinova = [ 0 0 0 pi/2 0 0 0 ]';
q0_puma = [ 0 0 0 0 0 0 ]';
q0_cell = {q0_panda q0_sawyer q0_ur5 q0_fanucM16i q0_kuka q0_abb q0_kinova q0_puma};
q0_s = q0_cell{robot_type};
figure
show(robot,q0_s);
%%
base_link_name = robot.BaseName; %"base"
end_link_name = end_link_names{robot_type}; % "end effector link"

pose0 = getTransform(robot,q0_s,end_link_name,base_link_name); % base frame is the frame to be expressed in 
initpos = pose0(1:3,4); % initial pose of end link expressed in the world frame

%% Get cartesian and joint trajectories

[pos, vel, acc] = getCartTraj(robot,q0_s, base_link_name, end_link_name, t_begin, t_end, Ts_p, shape_type,n,radius, alpha);

[pos_wpts, ~ , ~] = getCartTraj(robot,q0_s, base_link_name, end_link_name, t_begin, t_end, Ts_s, shape_type,n,radius, alpha);

[qd, dqd, ddqd, q_wpts] = getJointTraj(robot,q0_s, base_link_name, end_link_name,interploration_type,tvec_s, tvec_p, pos_wpts);

%% Make the animation of task trajectories and output GIF file 
if ~exist(path,'dir')
    mkdir(path);
end
addpath(genpath(path));
gif_filename = path  + save_name + ".gif";
visulizeTaskTraj (robot, tvec_p, qd, q0_s, end_link_name, Ts_ani, Ts_p, gif_filename);

%% Get the time series of final planed trajectoires and save it in results fold
qd = qd(1:dof,:);
dqd = dqd(1:dof,:);
ddqd = ddqd(1:dof,:);
traj.qd = timeseries(qd, tvec_p);
traj.dqd = timeseries(dqd, tvec_p);
traj.ddqd = timeseries(ddqd, tvec_p);
traj.pos = timeseries(pos, tvec_p);
traj.vel = timeseries(vel, tvec_p);
traj.acc = timeseries(acc, tvec_p);
mat_filename = path +save_name + ".mat";
save(mat_filename,"traj", '-mat')

% Plot the trajectories in joint space
plotJointSpaceTraj(tvec_p,qd,dqd,ddqd,tvec_s,q_wpts,path+save_name); % with waypoints
%plotJointSpaceTraj(tvec_p,qd,dqd,ddqd,[],[],path+save_name); % without waypoints

% Plot the trajectories in cartesian space
plotCartSpaceTraj(tvec_p,pos);





