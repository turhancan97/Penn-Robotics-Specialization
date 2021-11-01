function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here
persistent traj_time waypoints0 alpha time
if nargin > 2
    time = 1.5;
    waypoints0 = waypoints';
    n = size(waypoints0, 1) - 1;
    traj_time = [0];
    for i = 1:n
        traj_time = [traj_time, (i * time)];
    end
    
    A = [];
    b = [];
    
    for i = 1:n
        A = [A; zeros(1, (i - 1) * 8), 1, zeros(1, 7), zeros(1, (n - i) * 8)];
        b = [b; waypoints0(i, :)];
        A = [A; zeros(1, (i - 1) * 8), ones(1, 8), zeros(1, (n - i) * 8)];
        b = [b; waypoints0(i + 1, :)];
    end
    
    A = [A; 0, 1, zeros(1, 6), zeros(1, (n - 1) * 8)];
    A = [A; 0, 0, 1, zeros(1, 5), zeros(1, (n - 1) * 8)];
    A = [A; 0, 0, 0, 1, zeros(1, 4), zeros(1, (n - 1) * 8)];
    b = [b; 0, 0, 0];
    b = [b; 0, 0, 0];
    b = [b; 0, 0, 0];

    A = [A; zeros(1, (n - 1) * 8), 0, 1, 2, 3, 4, 5, 6, 7];
    A = [A; zeros(1, (n - 1) * 8), 0, 0, 2, 6, 12, 20, 30, 42];
    A = [A; zeros(1, (n - 1) * 8), 0, 0, 0, 6, 24, 60, 120, 210];
    b = [b; 0, 0, 0];
    b = [b; 0, 0, 0];
    b = [b; 0, 0, 0];

    for i = 2:n
        A = [A; zeros(1, (i - 2) * 8), 0, 1, 2, 3, 4, 5, 6, 7, 0, -1, zeros(1, 6), zeros(1, (n - i) * 8)];
        A = [A; zeros(1, (i - 2) * 8), 0, 0, 2, 6, 12, 20, 30, 42, 0, 0, -2, zeros(1, 5), zeros(1, (n - i) * 8)];
        A = [A; zeros(1, (i - 2) * 8), 0, 0, 0, 6, 24, 60, 120, 210, 0, 0, 0, -6, zeros(1, 4), zeros(1, (n - i) * 8)];
        A = [A; zeros(1, (i - 2) * 8), 0, 0, 0, 0, 24, 120, 360, 840, 0, 0, 0, 0, -24, zeros(1, 3), zeros(1, (n - i) * 8)];
        A = [A; zeros(1, (i - 2) * 8), 0, 0, 0, 0, 0, 120, 720, 2520, 0, 0, 0, 0, 0, -120, zeros(1, 2), zeros(1, (n - i) * 8)];
        A = [A; zeros(1, (i - 2) * 8), 0, 0, 0, 0, 0, 0, 720, 5040, 0, 0, 0, 0, 0, 0, -720, zeros(1, 1), zeros(1, (n - i) * 8)];
        b = [b; 0, 0, 0];
        b = [b; 0, 0, 0];
        b = [b; 0, 0, 0];
        b = [b; 0, 0, 0];
        b = [b; 0, 0, 0];
        b = [b; 0, 0, 0];
    end
    
    alpha = A \ b;
    
else
    if(t >= traj_time(end))
        desired_state.pos = waypoints0(end, :)';
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        t_index = find(traj_time <= t, 1, 'last');

        if(t == 0)
            desired_state.pos = waypoints0(1, :)';
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
        else
            if t_index > 1
                t = t - traj_time(t_index);
            end
            scale = t / time;
            coefficients = alpha((((t_index - 1) * 8) + 1):(t_index * 8), :);
            desired_state.pos = ([1, scale, scale^2, scale^3, scale^4, scale^5, scale^6, scale^7] * coefficients)';
            desired_state.vel = ([0, 1, 2*scale, 3*scale^2, 4*scale^3, 5*scale^4, 6*scale^5, 7*scale^6] * coefficients)';
            desired_state.acc = ([0, 0, 2, 6*scale, 12*scale^2, 20*scale^3, 30*scale^4, 42*scale^5] * coefficients)';
        end
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end
