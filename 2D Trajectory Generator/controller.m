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

k_vz = 20
k_pz = 500
k_vphi = 10
k_pphi = 400
k_vy = 5.5
k_py = 0.1


e_vz = des_state.vel(2)-state.vel(2)
e_pz = des_state.pos(2)-state.pos(2)

e_vy = des_state.vel(1)-state.vel(1)
e_py = des_state.pos(1)-state.pos(1)

phi_c = -(1/params.gravity)*(des_state.acc(1) + k_vy*e_vy + k_py*e_py)


e_vphi = -(state.omega)
e_pphi = phi_c - state.rot


u1 = params.mass*(params.gravity + des_state.acc(2) + k_vz*e_vz + k_pz*e_vz);
u2 = params.Ixx*(k_vphi*e_vphi+k_pphi*e_pphi)



end

