function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

Kp=130;
Kv=20;
z=s(1);
vz=s(2);
z_des=s_des(1);
vz_des=s_des(2);
g=params.gravity;
m=params.mass;
error=-z+z_des;
error_dot=-vz+vz_des;
z_double_dot_des=0;
u=m*(z_double_dot_des+(Kp*error)+(Kv*error_dot+g));
end

