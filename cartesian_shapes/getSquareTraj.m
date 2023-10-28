% Description:
%   This function gets the cartesian trajectories of a square shape within
%   specified time interval
% Input:
%   initpos: initial pose 
%   t: current time
%   t_begin: the start time of period
%   t_end: the end time of period
%   alpha (radius): angle between the square plane and XZ plane
%   radius: amplitude 
% Output:
%   pos: cartesian pose expressed in world frame
%   vel: cartesian velocity expressed in world frame (R_n^{0}v_n)
%   acc: cartesian acceleration expressed in wolrd frame (R_n^{0}a_n)

function [pos, vel, acc] = getSquareTraj(initpos,t,t_begin,t_end,alpha,radius)
        T = t_end-t_begin;
        ix = initpos(1);
        iy = initpos(2);
        iz = initpos(3);
        xi1 = ix;
        yi1 = iy;
        zi1 = iz;
        xi2 = xi1 - radius*cos(alpha);
        yi2 = yi1 - radius*sin(alpha);
        zi2 = zi1;
        xi3 = xi2;
        yi3 = yi2;
        zi3 = zi2 - radius;
        xi4 = xi3 + radius*cos(alpha);
        yi4 = yi3 + radius*sin(alpha);
        zi4 = zi3;
        if (t >= t_begin) &&(t-t_begin <= T/4)
            t = t-t_begin;
            phi = (sin(0.5*pi*(sin(0.5*pi*t*4/T))^2))^2;     
            phiDot  = (4*pi^2*cos((2*pi*t)/T)*sin((2*pi*t)/T)*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T;
            phiDotDot = 8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)^2/T^2 - (8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*sin((pi*sin((2*pi*t)/T)^2)/2)^2)/T^2 + (8*pi^3*cos((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2 - (8*pi^3*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2;
            rx = xi1 - radius*cos(alpha)*phi;
            ry = yi1 - radius*sin(alpha)*phi;
            rz = zi1;
            drx = -radius*cos(alpha)*phiDot;
            dry = -radius*sin(alpha)*phiDot;
            drz = 0;  
            ddrx = -radius*cos(alpha)*phiDotDot;
            ddry = -radius*sin(alpha)*phiDotDot;
            ddrz = 0;
        elseif (t-t_begin <= 2*T/4)
            t = t- T/4-t_begin;   
            phi = (sin(0.5*pi*(sin(0.5*pi*t*4/T))^2))^2;         
            phiDot  = (4*pi^2*cos((2*pi*t)/T)*sin((2*pi*t)/T)*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T;
            phiDotDot = 8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)^2/T^2 - (8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*sin((pi*sin((2*pi*t)/T)^2)/2)^2)/T^2 + (8*pi^3*cos((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2 - (8*pi^3*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2;
            rx = xi2;
            ry = yi2;
            rz = zi2 - radius*phi;
            drx = 0;
            dry = 0;
            drz = -radius*phiDot;
            ddrx = 0;
            ddry = 0;
            ddrz = -radius*phiDotDot;
        elseif (t-t_begin <= 3*T/4)
            t = t - 2*T/4-t_begin;
            phi = (sin(0.5*pi*(sin(0.5*pi*t*4/T))^2))^2;       
            phiDot  = (4*pi^2*cos((2*pi*t)/T)*sin((2*pi*t)/T)*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T;
            phiDotDot = 8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)^2/T^2 - (8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*sin((pi*sin((2*pi*t)/T)^2)/2)^2)/T^2 + (8*pi^3*cos((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2 - (8*pi^3*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2;
            rx = xi3 + radius*cos(alpha)*phi;
            ry = yi3 + radius*sin(alpha)*phi;
            rz = zi3;
            drx = radius*cos(alpha)*phiDot;
            dry = radius*sin(alpha)*phiDot;
            drz = 0;
            ddrx = radius*cos(alpha)*phiDotDot;
            ddry = radius*sin(alpha)*phiDotDot;
            ddrz = 0; 
        elseif (t-t_begin <= 4*T/4)
            t = t - 3*T/4-t_begin;
            phi = (sin(0.5*pi*(sin(0.5*pi*t*4/T))^2))^2;  
            phiDot  = (4*pi^2*cos((2*pi*t)/T)*sin((2*pi*t)/T)*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T;
            phiDotDot = 8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)^2/T^2 - (8*pi^4*cos((2*pi*t)/T)^2*sin((2*pi*t)/T)^2*sin((pi*sin((2*pi*t)/T)^2)/2)^2)/T^2 + (8*pi^3*cos((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2 - (8*pi^3*sin((2*pi*t)/T)^2*cos((pi*sin((2*pi*t)/T)^2)/2)*sin((pi*sin((2*pi*t)/T)^2)/2))/T^2;
            rx = xi4;
            ry = yi4;
            rz = zi4 + radius*phi;
            drx = 0;
            dry = 0;
            drz = radius*phiDot;
            ddrx = 0;
            ddry = 0;
            ddrz = radius*phiDotDot;
        else
            error('The current time reaches beyond the limits!');
        end
        pos = [rx;ry;rz];
        vel = [drx;dry;drz];
        acc = [ddrx;ddry;ddrz];
end