function [tarray, zarray, torque, P_hat, beta_hat, q_hat, w_meas, beta] = SIM(odefun,tspan,z0,n,p)
% Runge-Kutta 4th Order fixed-timestep integrator
%
% z = [q;w;qd;wd];
% modified to incorporate Linear Quadratic Gaussian Regulator

% initialization
% z = [q1 q2 q3 q4 wx wy wz q1d q2d q3d q4d wxd wyd wzd]
tarray = linspace(tspan(1),tspan(2),n+1);
dt = tarray(2) - tarray(1);
zarray = zeros(n+1,length(z0));
zarray(1,:) = z0'; 
torque = zeros(3, n);
beta = zeros(3, n+1); % gyro bias for sensor simulation

% parameters
I = p.I;

% LQR gains 
L1 = p.L1;
L2 = p.L2;

% noises
sigd = p.sigd; % control disturbance standard deviation

% struct for dynamics
d.I = I;
d.invI = p.invI;

% struct for measurement model
m.r = p.r;
m.sigv = p.sigv;
m.sigu = p.sigu;
m.sigq = p.sigq;
w_meas = zeros(3, n);

% struct for mekf
mekf.sigq = p.sigq;
mekf.r = p.r;
prop.sigv = p.sigv;
prop.sigu = p.sigu;
mekf.prop = prop;

% initialize estimates
P_hat = zeros(6, 6, n+1); % estimated error state covariance
P_hat(:, :, 1) = p.P0_hat; % initial error covariance
beta_hat = zeros(3, n+1); % estimated gyro bias
q_hat = zeros(4, n+1); % estimated quaternion
q_hat(:, 1) = p.q0_hat; % initial estimated quaternion


for i = 1:n
    % initial state (truth)
    z = zarray(i,:)';
    t = tarray(i);
    
    % Measurement Model (apply noise to gyroscope and attitude measurements)
    [w_tilda, b_tilda, betak_1] = measurement_model(z(5:7), z(1:4), m.r, dt, beta(:, i), m.sigv, m.sigu, m.sigq, p.dt_mekf, t);
    beta(:, i+1) = betak_1;
    w_meas(:, i) = w_tilda;
    
    % Multiplicative Extended Kalman Filter 
    % y (measurement = corrupted truth) => q_hat and w_hat (estimate)
    [q_plus, w_hat, P_plus, beta_plus] = MEKF(w_tilda, b_tilda, dt, q_hat(:, i), P_hat(:, :, i), beta_hat(:, i), mekf, t);
    q_hat(:, i+1) = q_plus;
    P_hat(:, :, i+1) = P_plus;
    beta_hat(:, i+1) = beta_plus;
    
    % controller
    if strcmp(p.controller, 'LQR')
        % Linear Quadratic Regulator
        T = LQR(q_plus, w_hat, z(8:11), z(12:14), I, L1, L2);
    elseif strcmp(p.controller, 'PD')
        % Nonlinear PD Controller
        T = PD(q_plus, w_hat, z(8:11), z(12:14), I, p.D, p.K);
    elseif strcmp(p.controller, 'none')
        % no controller
        T = zeros(3, 1);
    end
    torque(:, i) = T;
    
    % Actuator Model (apply noise to actuator)
    T = normrnd(T, sigd);
    d.T = T;
    
    % RK4 algorithm
    zarray(i+1,:) = RK4(odefun, t, z, d, dt);
    
    % normalize and properize quaternion
    zarray(i+1,1:4) = norm_prop(zarray(i+1,1:4));
    zarray(i+1,8:11) = norm_prop(zarray(i+1,8:11));
end
end