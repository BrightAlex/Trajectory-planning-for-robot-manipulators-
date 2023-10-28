% Description:
%   This function gets the cartesian trajectories of a fish shape within
%   specified time interval
% Input:
%   initpos: initial pose 
%   t: current time
%   t_begin: the start time of period
%   t_end: the end time of period
%   radius： amplitude of fish
% Output:
%   pos: cartesian pose expressed in world frame
%   vel: cartesian velocity expressed in world frame (R_n^{0}v_n)
%   acc: cartesian acceleration expressed in wolrd frame (R_n^{0}a_n)

function [pos, vel, acc] = getFishTraj(initpos,t,t_begin,t_end,radius)
        T = t_end-t_begin;  % time length of a period
        t = t-t_begin;
        
        w = 0.5*pi*(t)/T;
        wDot = 0.5*pi/T;
        z = sin(w);
        zDot = cos(w)*wDot;
        zDotDot = -sin(w)*wDot^2;
        y = sin(0.5*pi*z^2);
        yDot = pi*cos(0.5*pi*z^2)*z*zDot;
        yDotDot = -pi^2*z^2*zDot*sin(0.5*pi*z^2)*zDot + pi*cos(0.5*pi*z^2)*(zDot^2+z*zDotDot);
        theta = 2*pi*y^2;  
        thetaDot = 4*pi*y*yDot;
        thetaDotDot = 4*pi*(yDot^2+y*yDotDot);
        %theta = 2*pi*(sin(0.5*pi*(sin(0.5*pi*t/T))^2))^2;  
        %thetaDot = (2*pi^3*cos((pi*t)/(2*T))*sin((pi*t)/(2*T))*cos((pi*sin((pi*t)/(2*T))^2)/2)*sin((pi*sin((pi*t)/(2*T))^2)/2))/T;
        %thetaDotDot = (pi^5*cos((pi*t)/(2*T))^2*sin((pi*t)/(2*T))^2*cos((pi*sin((pi*t)/(2*T))^2)/2)^2)/T^2 - (pi^5*cos((pi*t)/(2*T))^2*sin((pi*t)/(2*T))^2*sin((pi*sin((pi*t)/(2*T))^2)/2)^2)/T^2 + (pi^4*cos((pi*t)/(2*T))^2*cos((pi*sin((pi*t)/(2*T))^2)/2)*sin((pi*sin((pi*t)/(2*T))^2)/2))/T^2 - (pi^4*sin((pi*t)/(2*T))^2*cos((pi*sin((pi*t)/(2*T))^2)/2)*sin((pi*sin((pi*t)/(2*T))^2)/2))/T^2;

        x = initpos(1);
        dx = 0;
        ddx = 0;
        y = radius*cos(theta) - radius*(sin(theta))^2/(2^0.5) + initpos(2)-radius;
        dy = (-radius*sin(theta) - radius*sin(2*theta)/(2^0.5))*thetaDot;
        ddy = -radius*(cos(theta)+2*cos(2*theta)/(2^0.5))*(thetaDot)^2+(-radius*sin(theta) - radius*sin(2*theta)/(2^0.5))*thetaDotDot;
        z = radius*cos(theta)*sin(theta) + initpos(3);
        dz = radius*cos(2*theta)*thetaDot;
        ddz = -2*radius*sin(2*theta)*(thetaDot)^2+radius*cos(2*theta)*thetaDotDot;
        pos = [x;y;z];
        vel = [dx;dy;dz];
        acc = [ddx;ddy;ddz];
end
