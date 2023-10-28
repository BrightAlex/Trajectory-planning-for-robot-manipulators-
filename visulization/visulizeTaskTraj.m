% Description:
%   This function visualizes the task-space trajectory by using the planed
%   joint positions and save the animation to GIF file
% Input:
%   robot: robot model 
%   tvec: time series
%   qd: planed joint positions nxP
%   q0_s: initial joint position nx1
%   Ts_ani:  time step for animation
%   filename: the path to save the GIF
% where n is the degree of freedom of robot, and P is the length of the
% times series.
% Reference:
%   see https://ww2.mathworks.cn/help/robotics/ug/plan-and-execute-trajectory-kinova-gen3.html

function visulizeTaskTraj (robot, tvec, qd, q0_s, end_link_name, Ts_ani, Ts_p, filename)
   %Show the initial configuration of the robot.
    fig = figure;
    show(robot,q0_s,'PreservePlot',false,'Frames','off');
    hold on
    axis([-1 1 -1 1 -0.1 1.5]);
    ind_gap = Ts_ani/Ts_p; 
    ind = 1:ind_gap:length(tvec);
    nImages = length(ind);
    im = cell(nImages,1);
    check_flag = 0;
    % Visualize the task-space trajectory
    for i=1:nImages
        poseNow = getTransform(robot,qd(:,ind(i)),end_link_name);
        show(robot,qd(:,ind(i)),'PreservePlot',false,'Frames','off','Visuals','on');
        plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',5);
        xlabel('X');ylabel('Y');zlabel('Z');
        drawnow;
        frame = getframe(fig);
        im{i} = frame2im(frame);
    end
    close;
    for idx = 1:nImages
        [A,map] = rgb2ind(im{idx},256);
        if check_flag == 0
            imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",Ts_ani);
            check_flag = 1;
        else
            imwrite(A,map,filename,"gif","WriteMode","append","DelayTime",Ts_ani);
        end
    end

end
