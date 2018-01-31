%% Attitude Simulator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~  Quaternion Convention  ~~~~~~~~~~~~~~~~~~~~~~~ %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% - spin axis (3x1):     e     = [qx; qy; qz] (must be unit vector)       %
% - rotation angle:      theta = angle of rotation from Inertial to Body  %
%                                Frame.                                   %
%                                                                         %
% - vector first, scalar last:                                            %
%   - q = [v; q4]                                                         %
%   - v = e.*sin(theta/2) = [q1; q2; q3]                                  %
%   - q4 = cos(theta/2)                                                   %
%                                                                         %
% - conjugate quaternion (inverse = conjugate)                            %
%   - q^-1 = [-v; q4]                                                     %
%   - (p x q)^-1 = q^-1 x p^-1                                            %
%                                                                         %
% - standard transformation                                               %
%   - [vp; 0] = q x [v; 0] x q^-1                                         %
%                                                                         %
% - left-handed convention (currently using)                              %
%   - i^2 = j^2 = k^2 = -1                                                %
%   - i = -jk; j = ik; k = -ij                                            %
%   - quaternion operator transforms a vector from Inertial to Body Frame %
%     coordinates                                                         %
%                                                                         %
% - right-handed convention                                               %
%   - i^2 = j^2 = k^2 = -1                                                %
%   - i = jk; j = -ik; k = ij                                             %
%   - quaternion operator transforms a vector from Body to Inertial Frame %
%     coordinates                                                         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close; clc;
addpath('ODE_Solver');
addpath('Quaternion');
addpath('Matrix');
addpath('Controller');
addpath('Dynamics');
addpath('Estimator');
addpath('Sensor Model');
addpath('Simulation');

% animation toggle 
animation = false; 
% slewing toggle
p.constant = true;
% choose controller ['LQR', 'PD', 'none']
p.controller = 'none';

% parameters
p.m = 6.26688; % mass [kg]
p.I = diag([20431045.58,...
            60163721.44,...
            66410423.47])./(1e9); %  moment of inertia matrix [kg*m^2]
p.invI = inv(p.I); % inverse of inertia matrix

% LQR gains (constant!)
A = [zeros(3, 3), eye(3); zeros(3, 6)];
B = [zeros(3, 3); eye(3)];
% penalties
Q = eye(6).*1e-2; 
R = eye(3);
[K,S,E] = lqr(A,B,Q,R);
L1 = K(:, 1:3);
L2 = K(:, 4:6);
p.L1 = L1;
p.L2 = L2;

% PD controller gains
p.K = eye(3).*1e-1;
p.D = eye(3).*5e-2;

% initial conditions
theta = deg2rad(20); spinaxis = [1;-1;0];
spinaxis = spinaxis./norm(spinaxis);
q0 = [spinaxis.*sin(theta/2); cos(theta/2)];
q0 = q0./norm(q0);
w0 = [0; 0; 6]; % [rad/s]
z0 = [q0; w0];

% MEKF gains
p.sigd = 0.000; % noise on disturbance torque
p.sigv = sqrt(10)*10^-4; % sqrt(10)*10^-7
p.sigu = sqrt(10)*10^-6; % sqrt(10)*10^-9
p.sigq = deg2rad(10)/3; % deg2rad(0.1)/3
p.r = reshape(eye(3), 9, 1); % inertial references
p.P0_hat = diag([3.0462e-6; 3.0462e-6; 3.0462e-6; 9.4018e-9;  9.4018e-9; 9.4018e-9].*10);
theta_hat = deg2rad(0); spinaxis_hat = [1;3;0];
spinaxis_hat = spinaxis_hat./norm(spinaxis_hat);
q0_hat = [spinaxis_hat.*sin(theta_hat/2); cos(theta_hat/2)];
q0_hat = q0_hat./norm(q0_hat);
p.q0_hat = q0;
p.dt_mekf = 2; % timestep of quaternion updates

% desired quaternion (reference) and angular velocities
if p.constant
    % constant reference
    thetad = deg2rad(-160); spinaxisd = [-1;3;-2]; 
    spinaxisd = spinaxisd./norm(spinaxisd);
    qd = [spinaxisd.*sin(thetad/2); cos(thetad/2)];
    qd = qd./norm(qd);
    wd = zeros(3, 1);
    z0 = [z0; qd; wd];
else
    % constant slew maneuver starting from initial quaternion 
    wd = [0.25; 0.25; -0.25];
    qd = q0;
    z0 = [z0; qd; wd];
end

% integraton
dt = 0.001;
tspanrk4 = [0, 300];
n = tspanrk4(2)/dt;
opts.RelTol = 1e-9;
opts.AbsTol = 1e-9;
[t, zarrayRK4, torque, P_hat, beta_hat, q_hat, w_meas, beta] = ...
    SIM(@dynamics,tspanrk4,z0,n,p);
% truths
q = zarrayRK4(:,1:4)'; 
w = zarrayRK4(:,5:7)';
qd = zarrayRK4(:,8:11)';
wd = zarrayRK4(:,12:14)';
% estimates
w_hat = [zeros(3,1), w_meas] - beta_hat;

% for constraints on state/input, penalize that and include in cost
% function%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% display data
display_data(animation, q, w, t, torque, p, dt, qd, wd, P_hat, w_hat, q_hat, beta_hat);
