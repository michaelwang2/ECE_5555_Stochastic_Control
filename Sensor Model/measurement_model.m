function [w_tilda, b_tilda, betak_1] = measurement_model(w, q, r, dt, betak, sigv, sigu, sigq, dt_mekf, t)
% discrete time measurement model for gyroscope and attitude sensor
betak_1 = betak + sigu*sqrt(dt)*normrnd(0, 1);
w_tilda = w + 0.5*(betak_1 + betak) + sqrt((sigv^2)/dt + (sigu^2)*dt/12)*normrnd(0, 1);
if rem(t, dt_mekf) == 0
    b_tilda = zeros(length(r), 1);
    for i = 1:(length(r)/3)
        b_tilda((3*i - 2):(3*i)) = (E(q)'*X(q))*r((3*i - 2):(3*i)) + normrnd(zeros(3,1), sigq);
    end
else
    b_tilda = nan;
end
end