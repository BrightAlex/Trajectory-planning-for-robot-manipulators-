% Description:
%   This function plots the the task trajectories including 3-D positions, velocities,
%   and accelerations in cartesian space. First three figures demonstrate the
%   trajectories over time and the last three figures demonstrate the
%   trajectories in 3-D cartesian coordinates.
% Input:
%   tvec: time series 1xP
%   pos: cartesian position 3xP
%   vel: cartesian velocity 3xP , optional
%   acc: cartesian acceleration 3xP , optional
%   pos_test: estimated position 3xP , optional
%   vel_test: estimated velocity 3xP, optional
% where P is the number of points in trajectories
function plotCartSpaceTraj(tvec,pos,vel,acc,pos_test, vel_test)

if nargin<=2
    % Positions in cartesian space
    figure
    plot3(pos(1,:),pos(2,:), pos(3,:),'r');
    ylabel('Y')
    xlabel('X')
    zlabel('Z')
    title('Position in cartesian space')

    % Comparison between planed positions and estimated positions over time 
    figure
    subplot(3,1,1)
    plot(tvec,pos(1,:),'r');
    if nargin <= 4
        legend('planed');
    else
        hold on;
        plot(tvec,pos_test(1,:),'b--');
        legend('planed','test')
    end
    title('Position coordinates')
    ylabel('X')
    subplot(3,1,2)
    plot(tvec,pos(2,:),'r');
    if nargin > 4
        hold on;
        plot(tvec,pos_test(2,:),'b--');
    end
    ylabel('Y')
    subplot(3,1,3)
    plot(tvec,pos(3,:),'r');
    if nargin>4
        hold on;
        plot(tvec,pos_test(3,:),'b--');
    end
    ylabel('Z')
    xlabel('Time (t)')
else

    % Comparison between planed velocities and estimated velocities over time 
    figure
    subplot(3,1,1)
    plot(tvec,vel(1,:),'r');
    if nargin <= 5
        legend('planed');
    else
        hold on;
        plot(tvec,vel_test(1,:),'b--');
        legend('planed','test')
    end
    title('Velocity coordinates')
    ylabel('X')
    subplot(3,1,2)
    plot(tvec,vel(2,:),'r');
    if nargin > 5
        hold on;
        plot(tvec,vel_test(2,:),'b--');
    end
    ylabel('Y')
    subplot(3,1,3)
    plot(tvec,vel(3,:),'r');
    if nargin > 5
        hold on;
        plot(tvec,vel_test(3,:),'b--');
    end
    ylabel('Z')
    xlabel('Time (t)')
    
    % Planed accelerations over time 
    figure
    subplot(3,1,1)
    plot(tvec,acc(1,:),'r');
    legend('planed')
    title('Accleration coordinates')
    ylabel('X')
    subplot(3,1,2)
    plot(tvec,acc(2,:),'r');
    ylabel('Y')
    subplot(3,1,3)
    plot(tvec,acc(3,:),'r');
    ylabel('Z')
    xlabel('Time (t)')
    
    % Comparison between planed position and estimated position in cartesian space
    figure
    plot3(pos(1,:),pos(2,:), pos(3,:),'r');
    if nargin <= 4
        legend('planed');
    else
        hold on;
        plot3(pos_test(1,:),pos_test(2,:), pos_test(3,:),'b--');
        legend('planed','test')
    end
    ylabel('Y')
    xlabel('X')
    zlabel('Z')
    title('Position in cartesian space')
    
    % Comparison between planed velocities and estimated velocities in cartesian space
    figure
    plot3(vel(1,:),vel(2,:), vel(3,:),'r');
    if nargin <=5
        legend('planed');
    else
        hold on;
        plot3(vel_test(1,:),vel_test(2,:), vel_test(3,:),'b--');
        legend('planed','test')
    end
    ylabel('Y')
    xlabel('X')
    zlabel('Z')
    title('Velocity in cartesian space')
    
    
    % Planed accelerations in cartesian space
    figure
    plot3(acc(1,:),acc(2,:), acc(3,:),'r');
    ylabel('Y')
    xlabel('X')
    zlabel('Z')
    title('Acceleration in cartesian space')
end
