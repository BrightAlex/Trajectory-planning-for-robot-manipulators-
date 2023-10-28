% Description:
% This scirpts verify the cartesian trajectories by integrating the acceleration and velocity
% using Euler method and compare the integration values with the velocity and the position, respectively,
% and plot the the cartesian position.
close all;
clc;
clear;
% get the inital cartesian position from specified joint configuration
robot = loadrobot('frankaEmikaPanda',DataFormat='column'); % including gripper 
q0_s = [0 0 0 -pi/2 0 pi/2 0 0 0]'; % initial configuration which could make z axis of end link vertial downward the ground 
pose0 = getTransform(robot,q0_s,'panda_link7','panda_link0'); % target frame is the frame to be expressed in 
initpos = pose0(1:3,4);
t_begin = 11; % start time point
T = 10; % period
t_end = t_begin+T; % end time point
Ts = 0.001; % sampling period
tvec = t_begin:Ts:t_end; % time series
num_traj_point = length(tvec); 
alpha = 30/180*pi; % angle between square face and XZ plane in square shape
radius = 0.1; % amplitude for 4 shape
n = 3; % petals for rose shape
pos = zeros(3,num_traj_point); % cartesian position
vel = zeros(3,num_traj_point); % cartesian velocity
acc = zeros(3,num_traj_point); % cartesian acceleration
pos_test = zeros(3,num_traj_point);
pos_test(:,1) = initpos;
vel_test = zeros(3,num_traj_point);

for i = 1:num_traj_point
    t = tvec(i);
    % choose shape here
    %[pos(:,i), vel(:,i), acc(:,i)] = getSquareTraj(initpos,t,t_begin,t_end,alpha,radius);
    %[pos(:,i), vel(:,i), acc(:,i)] = getFishTraj(initpos,t,t_begin,t_end,radius);
    %[pos(:,i), vel(:,i), acc(:,i)] = getRoseTraj(initpos,t,t_begin,t_end,radius,n);
    [pos(:,i), vel(:,i), acc(:,i)] = getInfinityLikeTraj(initpos,t,t_begin,t_end,radius);
    if i > 1
        vel_test(:,i) = vel_test(:,i-1) + Ts*acc(:,i); % integration value of acceleration
        pos_test(:,i) = pos_test(:,i-1) + Ts*vel(:,i); % integration value of velocity
    end
    checkValue('cartesian position',pos_test(i),pos(i),1e-2);
    checkValue('cartesian velocity',vel_test(i),vel(i),1e-2);
end

plotCartSpaceTraj(tvec,pos,vel,acc,pos_test,vel_test)

