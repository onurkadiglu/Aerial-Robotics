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

%% Main Function
persistent waypoints0 traj_time d0 coffx coffy coffz
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    coffx = getCoff(waypoints0(1,1:end))';
    coffy = getCoff(waypoints0(2,1:end))';
    coffz = getCoff(waypoints0(3,1:end))';
    
else
    if(t > traj_time(end))
        t = traj_time(end) - 0.0001;
    end
    
    t_index = find(traj_time >= t,1)-1;
    
    if (t_index == 0)
        t_index = 1;
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        scale = (t-traj_time(t_index))/d0(t_index);
        index = (t_index - 1) * 8 + 1 : t_index * 8;
        
        t0 = polyT(8, 0, scale)';
        t1 = polyT(8, 1, scale)';
        t2 = polyT(8, 2, scale)';
        
        desired_state.pos = [coffx(index) * t0; coffy(index) * t0; coffz(index) * t0];
        desired_state.vel = [coffx(index) * t1; coffy(index) * t1; coffz(index) * t1].*(1/d0(t_index));
        desired_state.acc = [coffx(index) * t2; coffy(index) * t2; coffz(index) * t2].*(1/d0(t_index)^2);
        
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

%% Coff Function
function [coff] = getCoff(waypoints)
    n = size(waypoints,2) - 1;
    A = zeros(8*n, 8*n);
    b = zeros(8*n, 1);
    
   %Const 1 Pi(0) = Wi for all i=1:n
    for i = 1:n
        A(i, 8*(i-1) + 1) = 1;
        b(i) = waypoints(i);
    end
    
    %Const 2 Pi(1) = Wi + 1 for all i=1:n
    for i = 1:n
        A(n + i, 8*(i-1) + 1 : 8*i) = 1;
        b(n+i) = waypoints(i+1);
    end
    
    %Const 3 P1(k)(0) = 0 for all k=1:3
    for k = 1:3
        A(2*n + k, 1:8) = polyT(8, k, 0);
    end
    
    %Const 4 Pn(k)(1) = 0 for all k=1:
    for k = 1:3
        A(2*n + 3 + k, 25:32) = polyT(8, k, 1);
    end
    
    %Const 5 Pi-1(k)(1) - Pi(k)(0) = 0 for all k=1:3
    for i = 1:n-1
        for k = 1:6
            A(2*n + 6*i + k, 8*(i-1) + 1 : 8*(i-1) + 16) = [polyT(8, k, 1), -1 * polyT(8, k, 0)];
        end
    end
    
    coff = inv(A) * b;
    
end

%% Util Function
function [E] = polyT(n, k, t)
    T = ones(1, n);
    D = zeros(1, n);
    for i = 1:n
        D(1,i) = i-1;
    end
    for d = 1:k
        for i = 1:n
            T(1,i) = T(1,i) * D(1,i);
            D(1,i) = max(0, D(1,i) - 1);
        end
    end
    E = T .* (t .^ D);
end

