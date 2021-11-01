function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

%Controller Parameters 
Kp_z=130;
Kv_z=20;
Kp_phi=1000;
Kv_phi=50;
Kp_y=40;
Kv_y=5;
%
y=state.pos(1);
z=state.pos(2);
y_dot=state.vel(1);
z_dot=state.vel(2);
phi=state.rot;
phi_dot=state.omega;
y_t=des_state.pos(1);
z_t=des_state.pos(2);
y_dot_t=des_state.vel(1);
z_dot_t=des_state.vel(2);
y_ddot_t=des_state.acc(1);
z_ddot_t=des_state.acc(2);
%Constant parameters
m=params.mass;
g=params.gravity;
Ixx=params.Ixx;
phi_ddot_t=0;
phi_dot_t=0;
%%%%%%%%%%%%%%%%%%%%%%%
%Equations of motion

u1=m*(g+z_ddot_t+Kv_z*(z_dot_t-z_dot)+Kp_z*(z_t-z))
phi_t=(-1/g)*(y_ddot_t+Kv_y*(y_dot_t-y_dot)+Kp_y*(y_t-y));
u2=Ixx*(phi_ddot_t+Kv_phi*(phi_dot_t-phi_dot)+Kp_phi*(phi_t-phi))

%%%%%%%%%%%%%%%%%%%%%%%%
end

