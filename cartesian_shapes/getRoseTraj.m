% Description:
%   This function gets the cartesian trajectories in shape of rose (also called a rosette, rosace, roseate or Rhodonea curve) in YZ
%   plane of the world frame, whose equations of pose are given by:
%       x = initpos(1)
%       r = a*sin(n*theta)
%       y = r*cos(theta)+initpos(2);
%       z = r*sin(theta)+initpos(3);
%   wher a is the constant amplitude of the shape, the variable n controls the number of petals:
%   Odd n = n-petalled.
%   Even n = 2n-petalled,
%   Irrational n = infinite petals.
% Input:
%   initpos: initial pose 
%   t: current time
%   t_begin: the start time of period
%   t_end: the end time of period
%   a: amplitude
%   n: the number of petals
%   rep: reptition times
% Output:
%   pos: cartesian pose expressed in world frame
%   vel: cartesian velocity expressed in world frame (R_n^{0}v_n)
%   acc: cartesian acceleration expressed in wolrd frame (R_n^{0}a_n)
% Reference:
%   https://www.statisticshowto.com/rhodonea-curve-rose/

function [pos, vel, acc, z, dz, ddz] = getRoseTraj(initpos,t,t_begin,t_end,a,n)  
        %   The θ, is the angle as a function of time;
        %   The dependent variable, r, is the radius as a function of θ.
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
        
        r = a*sin(n*theta);
        rDot = a*n*thetaDot*cos(n*theta);
        rDotDot = a*n*thetaDotDot*cos(n*theta) - a*n^2*thetaDot^2*sin(n*theta);
        x = initpos(1);
        dx = 0;
        ddx = 0;
        
        y = r*cos(theta)+initpos(2);
        dy = rDot*cos(theta) - r*sin(theta)*thetaDot;
        ddy = rDotDot*cos(theta) - rDot*sin(theta)*thetaDot - rDot*sin(theta)*thetaDot - r*(cos(theta)*thetaDot^2 + sin(theta)*thetaDotDot);
        
        z = r*sin(theta)+initpos(3);
        dz = rDot*sin(theta) + r*cos(theta)*thetaDot;
        ddz = rDotDot*sin(theta) + rDot*cos(theta)*thetaDot + rDot*cos(theta)*thetaDot + r*(-sin(theta)*thetaDot^2 + cos(theta)*thetaDotDot);
        
        pos = [x;y;z];
        vel = [dx;dy;dz];
        acc = [ddx;ddy;ddz];
end
