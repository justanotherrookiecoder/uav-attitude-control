
clear; clc;

%% Simulation settings
dt = 0.001;
T = 10;
t = 0:dt:T;
N = length(t);

%% UAV parameters
I0 = [0.020; 0.025];
b = [0.020; 0.020];
ref = deg2rad([10; -5]);
u_max = 0.5;

%% Fixed PID gains
Kp = [0.8; 0.8];
Ki = [0.05; 0.05];
Kd = [0.15; 0.15];

%% MPC settings
Np = 20;
Nc = 5;
Q = diag([100, 2]);
R = 0.5;
options = optimoptions('quadprog','Display','off');

%% Controller names
names = {'Fixed PID','Gain-Scheduled PID','MPC'};
nControllers = 3;

%% Results storage
angle_all = zeros(2,N,nControllers);
rate_all = zeros(2,N,nControllers);
torque_all = zeros(2,N,nControllers);
mse = zeros(2,nControllers);
peak_error = zeros(2,nControllers);
solver_success = zeros(1,nControllers);
solver_failure = zeros(1,nControllers);

%% Run each controller
for c = 1:nControllers

    angle = zeros(2,N);
    rate = zeros(2,N);
    torque = zeros(2,N);
    integral = zeros(2,1);

    for k = 1:N-1

        %% Inertia profile
        inertia = I0;

        if t(k) >= 4 && t(k) < 6
            inertia(1) = 1.30*I0(1);
        end

        if t(k) >= 6 && t(k) < 8
            inertia(2) = 1.20*I0(2);
        end

        %% Disturbances
        d = zeros(2,1);

        if t(k) >= 4 && t(k) < 4.5
            d(1) = 0.12;
        end

        if t(k) >= 6 && t(k) < 6.4
            d(2) = -0.12;
        end

        %% Current tracking error
        e = ref - angle(:,k);

        %% Controller
        if c == 1 || c == 2

            integral = integral + e*dt;
            integral = max(min(integral,2),-2);

            if c == 1
                Kp_now = Kp;
                Ki_now = Ki;
                Kd_now = Kd;
            else
                Kp_now = Kp.*(inertia./I0);
                Ki_now = Ki;
                Kd_now = Kd.*(inertia./I0);
            end

            u = Kp_now.*e + Ki_now.*integral ...
                - Kd_now.*rate(:,k);

            torque(:,k) = max(min(u,u_max),-u_max);

        else

            %% MPC for each axis
            for j = 1:2

                I = inertia(j);

                A = [1, dt;
                     0, 1-b(j)*dt/I];

                B = [0; dt/I];

                xj = [angle(j,k); rate(j,k)];
                rj = ref(j);

                F = zeros(2*Np,2);
                G = zeros(2*Np,Nc);

                for p = 1:Np
                    rows = 2*p-1:2*p;
                    F(rows,:) = A^p;

                    for m = 1:min(p,Nc)
                        G(rows,m) = A^(p-m)*B;
                    end
                end

                Xref = repmat([rj;0],Np,1);


                % MPC assumes future disturbance is unknown
                D = zeros(2*Np,1);

                Qbar = kron(eye(Np),Q);
                Rbar = R*eye(Nc);

                H = 2*(G'*Qbar*G + Rbar);
                H = (H+H')/2;
                f = 2*G'*Qbar*(F*xj + D - Xref);

                lb = -u_max*ones(Nc,1);
                ub =  u_max*ones(Nc,1);

                [U,~,exitflag] = quadprog( ...
                    H,f,[],[],[],[],lb,ub,[],options);

                if exitflag > 0 && ~isempty(U) ...
                        && all(isfinite(U))
                    solver_success(c) = solver_success(c)+1;
                    torque(j,k) = U(1);
                else
                    solver_failure(c) = solver_failure(c)+1;
                    torque(j,k) = 0;
                end
            end
        end

        %% Common plant dynamics
        acceleration = (torque(:,k)+d-b.*rate(:,k))./inertia;

        rate(:,k+1) = rate(:,k)+acceleration*dt;
        angle(:,k+1) = angle(:,k)+rate(:,k+1)*dt;
    end

    %% Store results
    angle_all(:,:,c) = angle;
    rate_all(:,:,c) = rate;
    torque_all(:,:,c) = torque;

    err_deg = rad2deg(ref-angle);

    rw = t >= 4 & t <= 5.5;
    pw = t >= 6 & t <= 7.5;

    mse(1,c) = mean(err_deg(1,rw).^2);
    mse(2,c) = mean(err_deg(2,pw).^2);

    peak_error(1,c) = max(abs(err_deg(1,rw)));
    peak_error(2,c) = max(abs(err_deg(2,pw)));
end

%% CONTROL-EFFORT METRICS
fprintf('\n===== CONTROL EFFORT =====\n');

for c = 1:nControllers
    fprintf('\n%s\n', names{c});

    for j = 1:2
        u = squeeze(torque_all(j,:,c));

        rms_torque = sqrt(mean(u.^2));
        peak_torque = max(abs(u));
        saturation_percent = 100 * mean(abs(u) >= u_max - 1e-8);

        if j == 1
            axis_name = 'Roll';
        else
            axis_name = 'Pitch';
        end

        fprintf('%s RMS torque: %.4f N.m\n', axis_name, rms_torque);
        fprintf('%s peak torque: %.4f N.m\n', axis_name, peak_torque);
        fprintf('%s saturation: %.2f%%\n', axis_name, saturation_percent);
    end
end

%% Print comparison
fprintf('\n===== CONTROLLER COMPARISON =====\n');

for c = 1:nControllers
    fprintf('\n%s\n',names{c});
    fprintf('Roll MSE: %.4f deg^2\n',mse(1,c));
    fprintf('Pitch MSE: %.4f deg^2\n',mse(2,c));
    fprintf('Roll peak error: %.4f deg\n',peak_error(1,c));
    fprintf('Pitch peak error: %.4f deg\n',peak_error(2,c));
    fprintf('Successful MPC solves: %d\n',solver_success(c));
    fprintf('Failed MPC solves: %d\n',solver_failure(c));
end

fprintf('\nSimulation complete.\n');
