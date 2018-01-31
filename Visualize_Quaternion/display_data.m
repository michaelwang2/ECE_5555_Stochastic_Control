function display_data(animation, q, w, t, torque, p, dt, qd, wd, P_hat, w_hat, q_hat, beta_hat)
if animation
    % animation 
    fig = figure; 
    ax = ['x','y','z'];
    for i = 1:3
        subplot(3, 2, 2*i); hold on;
        set(gca, 'FontSize', 13);
        if i == 1
            title(['$$Applied\ Torque\ in\ Body\ Coordinates\ (', p.controller, ')$$'],'fontweight','bold','FontSize',25,'Interpreter','latex');
        end
        plot(t(2:end), torque(i, :), 'k-','LineWidth', 2);
        grid on; box on;
        yy = ylabel(['$$\tau_{', ax(i),'}\ [Nm]$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.185, 0.375, 0], 'FontSize',25);
        hold off;
    end
    xlabel('$$Time\ [s]$$','Interpreter','latex');
    
    subplot(3,2,[1,3,5]);
    grid on; box on; axis equal; hold on;
    axis([-1.1,1.1,-1.1,1.1,-1.1,1.1]); 
    view(135,30);
    title('$$Attitude\ Simulation$$','fontweight','bold','FontSize',25,'Interpreter','latex');
    xlabel('$$X$$','Rotation',0,'Interpreter','latex');
    ylabel('$$Y$$','Rotation',0,'Interpreter','latex');
    zlabel('$$Z$$','Rotation',0,'Interpreter','latex');
    plot3([0 1.2],[0 0],[0 0],'k-','LineWidth',1.3);
    plot3([0 0],[0 1.2],[0 0],'k-','LineWidth',1.3);
    plot3([0 0],[0 0],[0 1.2],'k-','LineWidth',1.3); 

    pause(1);
    tic;
    cur_time = toc;
    while cur_time < t(end)
        % make quaternion so that it transforms from body coordinates to
        % inertial coordinates
        ind = round(cur_time/dt) + 1;
        q_I_B = conj_quat(q(:,ind));
        q_I_B_conj = q(:,ind);

        i_vec = mult_quat(q_I_B,mult_quat([1;0;0;0],q_I_B_conj));
        j_vec = mult_quat(q_I_B,mult_quat([0;1;0;0],q_I_B_conj));
        k_vec = mult_quat(q_I_B,mult_quat([0;0;1;0],q_I_B_conj));

        vec1 = plot3([0 i_vec(1)],[0 i_vec(2)],[0 i_vec(3)],'r-','LineWidth',3);
        vec2 = plot3([0 j_vec(1)],[0 j_vec(2)],[0 j_vec(3)],'g-','LineWidth',3);
        vec3 = plot3([0 k_vec(1)],[0 k_vec(2)],[0 k_vec(3)],'b-','LineWidth',3);
        
        q_I_Bd = conj_quat(qd(:,ind));
        q_I_B_conjd = qd(:,ind);

        i_vecd = mult_quat(q_I_Bd,mult_quat([1;0;0;0],q_I_B_conjd));
        j_vecd = mult_quat(q_I_Bd,mult_quat([0;1;0;0],q_I_B_conjd));
        k_vecd = mult_quat(q_I_Bd,mult_quat([0;0;1;0],q_I_B_conjd));

        vec1d = plot3([0 i_vecd(1)],[0 i_vecd(2)],[0 i_vecd(3)],'r-.','LineWidth',3);
        vec2d = plot3([0 j_vecd(1)],[0 j_vecd(2)],[0 j_vecd(3)],'g-.','LineWidth',3);
        vec3d = plot3([0 k_vecd(1)],[0 k_vecd(2)],[0 k_vecd(3)],'b-.','LineWidth',3);

        l = legend([vec1, vec2, vec3, vec1d, vec2d, vec3d], 'X', 'Y', 'Z', 'X_d', 'Y_d', 'Z_d', 'Location', 'Best');

        drawnow;
        delete(vec1);
        delete(vec2);
        delete(vec3);
        delete(vec1d);
        delete(vec2d);
        delete(vec3d);
        if ~ishghandle(fig)
            pause(0.25);
            close all;
            break;
        end
        cur_time = toc;
    end
    
else
    % plot telemetry
    fig = figure;
    subplot(3,2,2); hold on;
    set(gca, 'FontSize', 13);
    title('$$Angular\ Velocities\ in\ Body\ Coordinates$$','fontweight','bold','FontSize',25,'Interpreter','latex');
    plot(t,w(1,:),'k-','LineWidth', 2);  grid on; box on;
    y = ylabel('$$\omega_{x}\ \frac{rad}{s}$$','Interpreter','latex','Rotation',0);
    set(y, 'Units', 'Normalized', 'Position', [-0.175, 0.375, 0], 'FontSize',25); hold off;
    subplot(3,2,4); 
    set(gca, 'FontSize', 13);
    plot(t,w(2,:),'k-','LineWidth', 2); grid on; box on; 
    y = ylabel('$$\omega_{y}\ \frac{rad}{s}$$','Interpreter','latex','Rotation',0);
    set(y, 'Units', 'Normalized', 'Position', [-0.175, 0.375, 0], 'FontSize',25);
    subplot(3,2,6); 
    set(gca, 'FontSize', 13);
    plot(t,w(3,:),'k-','LineWidth', 2); grid on; box on;
    y = ylabel('$$\omega_{z}\ \frac{rad}{s}$$','Interpreter','latex','Rotation',0);
    set(y, 'Units', 'Normalized', 'Position', [-0.175, 0.375, 0], 'FontSize',25);
    xlabel('Time\ [s]','Interpreter','latex', 'FontSize', 25);
    ax = ['x','y','z'];
    for i = 1:3
        subplot(3, 2, 2*i-1); hold on;
        set(gca, 'FontSize', 13);
        if i == 1
            title(['$$Applied\ Torque\ in\ Body\ Coordinates\ (', p.controller, ')$$'],'fontweight','bold','FontSize',25,'Interpreter','latex');
        end
        plot(t(2:end), torque(i, :), 'k-','LineWidth', 2);
        grid on; box on;
        yy = ylabel(['$$\tau_{', ax(i),'}\ [Nm]$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.185, 0.375, 0], 'FontSize',25);
        hold off;
    end
    xlabel('$$Time\ [s]$$','Interpreter','latex', 'FontSize', 25);
    
    % error
    dq = zeros(4, length(t));
    dw = zeros(3, length(t));
    for j = 1:length(t)
        dq(:, j) = mult_quat(q(:, j), conj_quat(qd(:, j)));
        dw(:, j) = w(:, j) - wd(:, j);
    end

    figure; hold on;
    % dq = q*qd^-1
    % dw = w - wd
    for i = 1:4
        subplot(12, 2, [6*i-5, 6*i-3, 6*i-1]); hold on;
        plot(t,dq(i,:),'k-', 'LineWidth', 2); grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Error\ Quaternion\ (\delta q=q \otimes q_{d}^{-1})$$','fontweight','bold','FontSize',25,'Interpreter','latex');
        elseif i == 4
            xlabel('$$Time\ [s]$$','Interpreter','latex', 'FontSize', 25);
        end
        if any(i == [1,2,3])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$\delta q_{' num2str(i), '}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.165, 0.4, 0], 'FontSize', 25);
        hold off;
    end
    coords = 'xyz';
    for i = 1:3
        subplot(12, 2, [8*i-6, 8*i-4, 8*i-2, 8*i]); hold on;
        plot(t,dw(i,:),'k-', 'LineWidth', 2); grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Angular\ Velocity\ Error\ (\delta w=w\ -\ w_{d})$$','fontweight','bold','FontSize',25,'Interpreter','latex');
        elseif i == 3
            xlabel('$$Time\ [s]$$','Interpreter','latex','FontSize',25);
        end
        if any(i == [1,2])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$\delta w_{' coords(i), '}\ \frac{rad}{s}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.15, 0.4, 0],'FontSize',25);
        hold off;
    end
    
    % estimation error
    dq_hat = zeros(4, length(t));
    dw_hat = zeros(3, length(t));
    for j = 1:length(t)
        dq_hat(:, j) = mult_quat(q(:, j), conj_quat(q_hat(:, j)));
        dw_hat(:, j) = w(:, j) - w_hat(:, j);
    end
    
    figure; hold on;
    % dq_hat = q*q_hat^-1
    % dw_hat = w - w_hat
    for i = 1:4
        subplot(12, 2, [6*i-5, 6*i-3, 6*i-1]); hold on;
        plot(t,dq_hat(i,:),'k-', 'LineWidth', 2); grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Quaternion\ Estimation\ Error\ (\delta \hat{q}=q \otimes \hat{q}^{-1})$$','fontweight','bold','FontSize',25,'Interpreter','latex');
        elseif i == 4
            xlabel('$$Time\ [s]$$','Interpreter','latex', 'FontSize', 25);
        end
        if any(i == [1,2,3])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$\delta \hat{q}_{' num2str(i), '}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.17, 0.4, 0], 'FontSize', 25);
        hold off;
    end
    coords = 'xyz';
    for i = 1:3
        subplot(12, 2, [8*i-6, 8*i-4, 8*i-2, 8*i]); hold on;
        plot(t,dw_hat(i,:),'k-', 'LineWidth', 2); grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Angular\ Velocity\ Estimation\ Error\ (\delta \hat{w}=w\ -\ \hat{w})$$','fontweight','bold','FontSize',25,'Interpreter','latex');
        elseif i == 3
            xlabel('$$Time\ [s]$$','Interpreter','latex','FontSize',25);
        end
        if any(i == [1,2])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$\delta \hat{w}_{' coords(i), '}\ \frac{rad}{s}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.15, 0.4, 0],'FontSize',25);
        hold off;
    end
    
    figure; hold on;
    for i = 1:4
        subplot(12, 2, [6*i-5, 6*i-3, 6*i-1]); hold on;
        est = plot(t,q_hat(i,:),'b-o','LineWidth', 2, 'MarkerSize', 3);
        truth = plot(t,q(i,:),'k-', 'LineWidth', 2); 
        grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Quaternion\ Estimation\ $$','fontweight','bold','FontSize',25,'Interpreter','latex');
            legend([truth, est], 'Truth', 'Estimate');
        elseif i == 4
            xlabel('$$Time\ [s]$$','Interpreter','latex', 'FontSize', 25);
        end
        if any(i == [1,2,3])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$q_{' num2str(i), '}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.17, 0.4, 0], 'FontSize', 25);
        hold off;
    end
    coords = 'xyz';
    for i = 1:3
        subplot(12, 2, [8*i-6, 8*i-4, 8*i-2, 8*i]); hold on;
        est = plot(t,w_hat(i,:),'b-o','LineWidth', 2, 'MarkerSize', 3);
        truth = plot(t,w(i,:),'k-', 'LineWidth', 2); 
        grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Angular\ Velocity\ Estimation\ $$','fontweight','bold','FontSize',25,'Interpreter','latex');
            legend([truth, est], 'Truth', 'Estimate');
        elseif i == 3
            xlabel('$$Time\ [s]$$','Interpreter','latex','FontSize',25);
        end
        if any(i == [1,2])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$w_{' coords(i), '}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.15, 0.4, 0],'FontSize',25);
        hold off;
    end
    
    figure; hold on;
    % gyro drift estimates
    coords = 'xyz';
    for i = 1:3
        subplot(3, 1, i); hold on;
        plot(t,beta_hat(i,:),'k-', 'LineWidth', 2); grid on; box on;
        set(gca, 'FontSize', 13);
        if i == 1
            title('$$Gyro\ Drift\ Estimation$$','fontweight','bold','FontSize',25,'Interpreter','latex');
        elseif i == 3
            xlabel('$$Time\ [s]$$','Interpreter','latex','FontSize',25);
        end
        if any(i == [1,2])
            set(gca, 'XTickLabel', []);
        end
        yy = ylabel(['$$\hat{\beta}_{',coords(i),'}$$'],'Interpreter','latex', 'Rotation', 0);
        set(yy, 'Units', 'Normalized', 'Position', [-0.07, 0.4, 0],'FontSize',25);
        hold off;
    end
end
end