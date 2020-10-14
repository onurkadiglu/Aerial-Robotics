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

% Variable initialization
m = params.mass;
g = params.gravity;
I = params.I;
inv_I = params.invI;
L = params.arm_length;

x = state.pos(1);
y = state.pos(2);
z = state.pos(3);
x_dot = state.vel(1);
y_dot = state.vel(2);
z_dot = state.vel(3);
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

x_des = des_state.pos(1);
y_des = des_state.pos(2);
z_des = des_state.pos(3);
x_des_dot = des_state.vel(1);
y_des_dot = des_state.vel(2);
z_des_dot = des_state.vel(3);
x_des_ddot = des_state.acc(1);
y_des_ddot = des_state.acc(2);
z_des_ddot = des_state.acc(3);
psi_des = des_state.yaw;
psi_des_dot = des_state.yawdot;

% Constants
kd_x = 50;
kp_x = 3;
kd_y = 50;
kp_y = 3;
kp_z = 800;
kd_z = 40;
kp_phi = 200;
kd_phi = 3;
kp_theta = 200;
kd_theta = 3;
kp_psi = 200;
kd_psi = 3;

% Commanded accelerations
r1_des_ddot_x = x_des_ddot + kd_x * (x_des_dot - x_dot) + kp_x * (x_des - x);
r2_des_ddot_y = y_des_ddot + kd_y * (y_des_dot - y_dot) + kp_y * (y_des - y);
r3_des_ddot_z = z_des_ddot + kd_z * (z_des_dot - z_dot) + kp_z * (z_des - z);

% Thrust
F = m * (g + r3_des_ddot_z);

% Moment
p_des = 0;
q_des = 0;
r_des = psi_des_dot;

phi_des = 1/g * (r1_des_ddot_x * sin(psi_des) - r2_des_ddot_y * cos(psi_des));
theta_des = 1/g * (r1_des_ddot_x * cos(psi_des) + r2_des_ddot_y * sin(psi_des));

M = [kp_phi * (phi_des - phi) + kd_phi * (p_des - p);
     kp_theta * (theta_des - theta) + kd_theta * (q_des - q);
     kp_psi * (psi_des - psi) + kd_psi * (r_des - r)];
% =================== Your code ends here ===================

end