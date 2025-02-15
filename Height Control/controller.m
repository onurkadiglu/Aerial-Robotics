function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

z2des = 0
Kp = 100
Kv = 15
e = s_des(1)-s(1);
e1 = s_des(2)-s(2);
u = params.mass*(z2des + Kp*e + Kv*e1 + params.gravity);


end

