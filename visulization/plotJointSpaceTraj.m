% Description:
%   This function plots the trajectories including n-D positions, velocities,
%   and accelerations in joint space, where n is the degrees of freedom of robot. 
% Input:
%   t: time series 1xP of planed trajectories
%   q: cartesian position 3xP 
%   dq: cartesian velocity 3xP
%   ddq: cartesian acceleration 3xP
%   wpts: waypoints to obtain these trajectories before interpolation nxS , optional
%   tpts: time series of waypoints 1xS, optional
%   path_name: path to save the figure
%   
% where P is the number of points in interpolated trajectories, S is the
% number of waypoints 
function plotJointSpaceTraj(t,q,dq,ddq,tpts,wpts,path_name)
% Plot results
figure;
% Plot joint positions 
subplot(3,1,1);
plot(t,q);
if exist('wpts','var') && exist('tpts','var')
    hold all
    plot(tpts, wpts, 'x', 'MarkerSize', 7,'LineWidth',2);
end
xlabel('Time(s)')
ylim('padded');
title('Joint-Space Trajectory');



% Plot joint velocities
subplot(3,1,2);
plot(t,dq)
xlabel('Time(s)')
ylim('padded');
title('Joint-Space Velocities');


% Plot joint accelerations
subplot(3,1,3);
plot(t,ddq)
xlabel('Time(s)')
ylim('padded');
title('Joint-Space Acceleration');

if exist("path_name",'var')
    exportgraphics(gcf,path_name + '_JointTraj' + '.jpg','Resolution',600)
    % exportgraphics(gcf,filename + '.eps','ContentType','image')
    % exportgraphics(gcf,filename + '.pdf','ContentType','image')
end
end

