% Description:
%   This function gets the joint and cartesian trajectories in a specified
%   shape for robot 
% Input:
%   robot: robot object retured by function, loadrobot
%   q0_s: initial cartesian pose 9x1 
%   base_link_name: the name of the base link
%   end_link_name: the name of the end link
%   shape_type : speficy the type of shape

% Output:
%   q: joint position nxP
%   dq: joint velocity nxP
%   ddq: joint acceleration nxP
% where P is the number of points in final trajectories

function [qd, dqd, ddqd, q_wpts] = getJointTraj(robot,q0_s, base_link_name, end_link_name,interploration_type,tvec_s, tvec_p, pos_sample)
    pose0 = getTransform(robot,q0_s,end_link_name,base_link_name); % target frame is the frame to be expressed in
    eul0 =  tform2eul(pose0); % Euler angle of end frame with respecet to world frame 
    S = length(tvec_s); % number of sampling points, namely waypoints
    P = length(tvec_p); % number of points in final trajectories
    disp('*************************** begin IK ********************************')
    ik = inverseKinematics(RigidBodyTree=robot);
    ik.SolverParameters.AllowRandomRestart = false;
    q_wpts = zeros(size(q0_s,1),S);
    weights = [0.1 0.1 0.1 1 1 1]; % Prioritize position over orientation
    initialGuess = q0_s; % Choose an inital guess within the robot joint limit
    for i = 1:S
        fprintf("IK index: %d \n", i);
        targetPose = trvec2tform(pos_sample(:,i)')*eul2tform(eul0); % used to keep euler angle of end effector static
        q_wpts(:,i) = ik(end_link_name,targetPose,weights,initialGuess);
        initialGuess = q_wpts(:,i); % Use the last result as the next initial guess
    end
    disp('*************************** end IK ********************************')
    disp('*************************** begin interpolation ********************************')
    if interploration_type == 1
        % Minimum-Jerk Trajectory
        [qd,dqd,ddqd] = minjerkpolytraj(q_wpts,tvec_s,P); % jerk means derivative of acceleration
    elseif interploration_type ==2
        % Trapezoidal Velocity Profile Trajectory
        [qd,dqd,ddqd] = trapveltraj(q_wpts,P);
    elseif interploration_type == 3
        % Cubic and Quintic Polynomial Trajectories
        [qd,dqd,ddqd] = cubicpolytraj(q_wpts,tvec_s,tvec_p);
    elseif interploration_type == 4
        % B-Spline Trajectory
        [qd,dqd,ddqd] = bsplinepolytraj(q_wpts,tvec_s([1 end]),tvec_p);
    else
        error("Please input the correct number for interploration type!");
    end
    disp('*************************** end interpolation ********************************')
end
