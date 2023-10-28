% Description:
%   This function gets the cartesian trajectories in a infinity-like shape
%   composed of two circle in XY plane of the world frame within specified time interval, whose equations of pose are given by:
%           first circle: 
%               center c1: (x0,r,z0)
%               vector r1: from c1 to current trajectory point on the first circle
%               theta: angle between r1 and -Y axis 
%               x = r*sin(theta) + x0;
%               y = r-r*cos(theta) + y0
%               z = z0;
%           second circle:
%               center c2: (x0,-r,z0)
%               vector r2: from c2 to current trajectory point on the second circle
%               theta: angle between r2 and Y axis 
%               x = r*sin(theta) + x0;
%               y = r*cos(theta)-r + y0
%               z = z0;
%   where r is the constant radius of the circle , (x0, y0, z0) is the coordinate of
%   the initial position.

% Input:
%   initpos: initial position 
%   t: current time
%   t_begin: the start time of period
%   t_end: the end time of period
%   r: radius of the circle 
% Output:
%   pos: cartesian pose expressed in world frame
%   vel: cartesian velocity expressed in world frame (R_n^{0}v_n)
%   acc: cartesian acceleration expressed in wolrd frame (R_n^{0}a_n)

function [pos, vel, acc] = getInfinityLikeTraj(initpos,t,t_begin,t_end,r) 
        T = (t_end-t_begin)/2;
        % theta = 2*pi*(sin(0.5*pi*(sin(0.5*pi*t/T))^2))^2;       
        % thetaDot  = (2*pi^3*cos((pi*t)/(2*T))*sin((pi*t)/(2*T))*cos((pi*sin((pi*t)/(2*T))^2)/2)*sin((pi*sin((pi*t)/(2*T))^2)/2))/T;
        % thetaDotDot = (pi^5*cos((pi*t)/(2*T))^2*sin((pi*t)/(2*T))^2*cos((pi*sin((pi*t)/(2*T))^2)/2)^2)/T^2 - (pi^5*cos((pi*t)/(2*T))^2*sin((pi*t)/(2*T))^2*sin((pi*sin((pi*t)/(2*T))^2)/2)^2)/T^2 + (pi^4*cos((pi*t)/(2*T))^2*cos((pi*sin((pi*t)/(2*T))^2)/2)*sin((pi*sin((pi*t)/(2*T))^2)/2))/T^2 - (pi^4*sin((pi*t)/(2*T))^2*cos((pi*sin((pi*t)/(2*T))^2)/2)*sin((pi*sin((pi*t)/(2*T))^2)/2))/T^2;
        if t >= t_begin && t <= T + t_begin
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

            y = r-r*cos(theta) + initpos(2);
            dy = r*sin(theta)*thetaDot;
            ddy = r*cos(theta)*thetaDot^2+r*sin(theta)*thetaDotDot;
        elseif t  <= t_end
            t = t-t_begin-T;
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

            y = r*cos(theta)-r+initpos(2);
            dy = -r*sin(theta)*thetaDot;
            ddy = -r*cos(theta)*thetaDot^2-r*sin(theta)*thetaDotDot;
        else
            error("The current time reaches beyond the limits!");
        end
        x = r*sin(theta)+initpos(1);
        dx = r*cos(theta)*thetaDot;
        ddx = -r*sin(theta)*thetaDot^2+r*cos(theta)*thetaDotDot;
        z = initpos(3);
        dz = 0;
        ddz = 0;
        pos = [x;y;z];
        vel = [dx;dy;dz];
        acc = [ddx;ddy;ddz];
end
