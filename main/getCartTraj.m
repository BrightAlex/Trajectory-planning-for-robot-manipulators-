% Description:
%   This function gets the cartesian trajectories in a specified
%   shape for robot 
% Input:
%   robot: robot object retured by function, loadrobot
%   q0_s: initial cartesian pose 9x1 
%   base_link_name: the name of the base link
%   end_link_name: the name of the end link
%   t_beg: the start time, scalar
%   T: length of time interval, scalar
%   Ts:   the period of the planed joint trajectories 
%   shape_type : speficy the type of shape

% Output:
%   pos: cartesian position expressed in world frame 3xP
%   vel: cartesian velocity expressed in world frame 3xP
%   acc: cartesian acceleration expressed in world frame 3xP 
% where P is the number of points in planned trajectories

function [pos, vel, acc] = getCartTraj(robot,q0_s, base_link_name, end_link_name, t_begin, t_end, Ts, shape_type,n,radius, alpha) 
    pose0 = getTransform(robot,q0_s,end_link_name,base_link_name); % target frame is the frame to be expressed in
    initpos = pose0(1:3,4); % initial position
    tvec_p = t_begin:Ts:t_end;
    P = length(tvec_p); % number of points in final trajectories
    pos = zeros(3,P);
    vel = zeros(3,P);
    acc = zeros(3,P);
    i = 1;
    for t = tvec_p
        % choose shape here
        if shape_type == 1 
            [pos(:,i), vel(:,i), acc(:,i)] = getSquareTraj(initpos,t,t_begin,t_end,alpha,radius);
        elseif shape_type == 2
            [pos(:,i), vel(:,i), acc(:,i)] = getFishTraj(initpos,t,t_begin,t_end,radius);
        elseif shape_type == 3
            [pos(:,i), vel(:,i), acc(:,i)] = getRoseTraj(initpos,t,t_begin,t_end,radius,n);
        elseif shape_type == 4
            [pos(:,i), vel(:,i), acc(:,i)] = getInfinityLikeTraj(initpos,t,t_begin,t_end,radius);
        else
            error('Please input correct number of shape type');
        end
        i = i+1;
    end
end