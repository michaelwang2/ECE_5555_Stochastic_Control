function [q_plus, w_hat, P_plus, beta_plus] = MEKF(w_tilda, b_tilda, dt, q_prev, P_prev, beta_prev, mekf, t)
% Multiplicative Quaternion Extended Kalman Filter

% RK4 algorithm to propagate state (a priori)
w_hat = w_tilda - beta_prev; mekf.prop.w_hat = w_hat;
z = [reshape(P_prev, 36, 1); q_prev; beta_prev];
z1 = RK4(@propagate, t, z, mekf.prop, dt);
P_minus = reshape(z1(1:36), 6, 6);
q_minus = norm_prop(z1(37:40));
beta_minus = z1(41:43);

if isnan(b_tilda)
    % if there are no measurements available
    q_plus = q_minus;
    P_plus = P_minus;
    beta_plus = beta_minus;
else
    % Update step (a posteori)
    r = mekf.r; % attitude references
    sigq = mekf.sigq;
    rvec = ones(length(r), 1).*sigq^2;
    R = diag(rvec); % quaternion measurement covariance matrix
    H = zeros(length(r), 6); % sensitivity matrix
    h = zeros(length(r), 1); % expected measurement
    for i = 1:(length(r)/3)
        H((3*i-2):(3*i), :) = [skew(E(q_minus)'*X(q_minus)*r((3*i-2):(3*i))), zeros(3, 3)]; 
        h((3*i-2):(3*i)) = E(q_minus)'*X(q_minus)*r((3*i-2):(3*i));
    end
    K = P_minus*H'*(H*P_minus*H' + R)^-1; % Kalman Gain
    P_plus = (eye(6) - K*H)*P_minus;
    dx_plus = K*(b_tilda - h); % dx_plus = [dalpha_plus; dbeta_plus];
    q_plus = q_minus + 0.5*E(q_minus)*dx_plus(1:3);
    q_plus = norm_prop(q_plus); % normalize and properize quaternion
    beta_plus = beta_minus + dx_plus(4:6);
end
end

function zdot = propagate(t, z, prop)
% z = [reshape(P_prev, 36, 1); q_prev; beta_prev];
% zdot = [reshape(P_dot, 36, 1); q_dot; beta_dot];
% unpack z
P = reshape(z(1:36), 6, 6);
q = z(37:40);
beta = z(41:43);

% parameters
w_hat = prop.w_hat;
sigv = prop.sigv;
sigu = prop.sigu;

F = [-skew(w_hat), -eye(3); zeros(3, 6)];
G = [-eye(3), zeros(3, 3); zeros(3, 3), eye(3)];
Q = [eye(3).*sigv^2, zeros(3, 3); zeros(3, 3), eye(3).*sigu^2];

P_dot = F*P + P*F' + G*Q*G';
q_dot = 0.5*E(q)*w_hat;
beta_dot = zeros(3, 1);

zdot = [reshape(P_dot, 36, 1); q_dot; beta_dot];
end