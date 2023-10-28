% Description:
% This scirpt verifies the operation of inverse kinematics and interploration in "getJointCartTraj.m" by applying the forward kinametics to the obtained joint trajectories and compare
% the result values with the planed cartesian trajectories
close all;
clc;
clear;

robot_type = 1;
shape_type = 1; % square:1, fish:2, rose:3, infinitylike:4
degree = -30;
alpha = degree/180*pi; % angle between square face and XZ plane in square shape
radius = 0.3; % amplitude for 4 shapes
n = 3; % petals for rose shape
interpolation_type = 1; % Minimum-Jerk Trajectory:1, Trapezoidal Velocity Profile Trajectory:2,  Cubic and Quintic Polynomial Trajectories:3, B-Spline Trajectory:4

t_begin = 0; % start time of trajectory (s)
T = 5; % period of the trajectory (s)
t_end = t_begin + T; % end time of trjectory (s)
Ts_s = 0.05; % sampling period of waypoints to be used for inverse kinematics (s)
Ts_p = 0.001; % period of final trajectories (s)
Ts_ani = 0.01; % sampling period of the visulization

robot_names = {"FrankaEmikaPanda", "rethinkSawyer", "universalUR5", "fanucM16i", "kukaIiwa7", 'abbIrb1600', "kinovaGen3", "puma560"};
shape_names = {"square", "fish", "rose", "infinity"};
interpolation_names = {"Minimum-Jerk", "Trapezoidal Velocity", "Cubic and Quintic Polynomial", "B-Spline"};
end_link_names = {"panda_link7", "right_l6"}; % to be added by user
dofs = {7, 8}; % the degrees of freedom,  to be added by user
dof = dofs{robot_type};
q0_panda = [0 0 0 -pi/2 0 pi/2 0 0 0]'; % initial joint position, which could make z axis of end link vertial downward the ground
q0_sawyer = [0 0 -pi/3 0 pi/3 0 pi/2 0]';
q0_ur5 = []';
q0_fanucM16i = []';
q0_kuka = [0 0 -pi/2 -pi/2 0 pi/2]';
q0_abb = []';
q0_kinova = []';
q0_puma = []';
q0_cell = {q0_panda q0_sawyer q0_ur5 q0_fanucM16i q0_kuka q0_abb q0_kinova q0_puma};
robot = loadrobot(robot_names{robot_type},DataFormat='column'); 
q0_s = q0_cell{robot_type};
figure
show(robot,q0_s);
%%
base_name = robot.BaseName; %"base"
end_link_name = end_link_names{robot_type}; % "end effector link"

pose0 = getTransform(robot,q0_s,end_link_name,base_name); % base frame is the frame to be expressed in 
initpos = pose0(1:3,4); % initial pose of end link expressed in the world frame

%% Get cartesian and joint trajectories 
[qd, dqd, ddqd ,pos, vel, acc, tvec] = getJointCartTraj(robot,q0_s,base_name,end_link_name,t_begin,t_end,Ts_s,Ts_p,interpolation_type,shape_type,n,radius,alpha);

%% Test cartesian positions and velocities by integrations 
indices = 1:length(tvec);
for i = 1:numel(indices)
   pose1 = getTransform(robot,qd(:,i),end_link_name,base_name);
   pos1 = pose1(1:3,4);
   pos2 = pos(:,i);
   % see https://ww2.mathworks.cn/help/robotics/ref/rigidbodytree.geometricjacobian.html
   Jac = geometricJacobian(robot,qd(:,i),end_link_name);
   vel_twist = Jac*dqd(:,i); 
   vel1 = vel_twist(4:6);
   vel2 =  vel(:,i);
   checkValue('check cartesian postion',pos1, pos2, 0.1);
   checkValue('check cartesian velocity', vel1, vel2, 0.1);
end





