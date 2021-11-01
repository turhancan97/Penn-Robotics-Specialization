function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

kprot = [25, 25, 0]';
    kdrot = [0.3, 0.3, 0]';
    kp = [250, 250, 250]';
    kd = [12.5, 12.5, 12.5]';

    m = params.mass;
    g = params.gravity;
        
    function com_acc = hover_com_acc()
        com_acc = -((kd .* state.vel) + (kp .* (state.pos - des_state.pos)));
    end
    
    function com_acc = trajectory_com_acc()
%         t_hat = des_state.vel / norm(des_state.vel);
%         n_hat = des_state.acc / norm(des_state.acc);
%         b_hat = cross(t_hat, n_hat);
%         e_p = (dot((des_state.pos - state.pos), n_hat) * n_hat + (dot((des_state.pos - state.pos), b_hat)) * b_hat);
%         e_v = des_state.vel - state.vel;
        com_acc = des_state.acc + (kd .* (des_state.vel - state.vel)) + (kp .* (des_state.pos - state.pos));
    end

    function com_acc = commanded_acceleration()
        if mod(t, 10) == 0
            com_acc = trajectory_com_acc();
        else
            com_acc = hover_com_acc();
        end
    end
    
    function F = thrust(com_acc)
        F = m * g + m * com_acc(3);
    end

    function rotdes = desired_rotation(com_acc)
        yawdes = des_state.yaw;
        rotdes = [(com_acc(1) * sin(yawdes) - com_acc(2) * cos(yawdes)) / g
                  (com_acc(1) * cos(yawdes) + com_acc(2) * sin(yawdes)) / g
                  yawdes];
    end

    com_acc = commanded_acceleration();
    
    % Thrust
    F = thrust(com_acc);
    
    % Moment
    rotdes = desired_rotation(com_acc);
    omegades = [0, 0, des_state.yawdot]';
    M = kprot .* (rotdes - state.rot) + kdrot .* (omegades - state.omega);

% =================== Your code ends here ===================

end
