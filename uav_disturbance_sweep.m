
clear; clc;

%% Simulation settings
dt = 0.001;
T = 10;
t = 0:dt:T;
N = length(t);

I0 = [0.020; 0.025];
b = [0.020; 0.020];
ref = deg2rad([10; -5]);
u_max = 0.5;

Kp = [0.8; 0.8];
Ki = [0.05; 0.05];
Kd = [0.15; 0.15];

Np = 20;
Nc = 5;
Q = diag([100, 2]);
R = 0.5;
options = optimoptions('quadprog','Display','off');

disturbance_levels = [0.08, 0.12, 0.16];
names = {'Fixed PID','Gain-Scheduled PID','MPC'};

results = [];

for s = 1:length(disturbance_levels)

    disturbance_strength = disturbance_levels(s);

    for c = 1:3

        angle = zeros(2,N);
        rate = zeros(2,N);
        torque = zeros(2,N);
        integral = zeros(2,1);

        solver_success = 0;
        solver_failure = 0;

        for k = 1:N-1

            inertia = I0;

            if t(k) >= 4 && t(k) < 6
                inertia(1) = 1.30*I0(1);
            end

            if t(k) >= 6 && t(k) < 8
                inertia(2) = 1.20*I0(2);
            end

            d = zeros(2,1);

            if t(k) >= 4 && t(k) < 4.5
                d(1) = disturbance_strength;
            end

            if t(k) >= 6 && t(k) < 6.4
                d(2) = -disturbance_strength;
            end

            e = ref - angle(:,k);

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
                        solver_success = solver_success + 1;
                        torque(j,k) = U(1);
                    else
                        solver_failure = solver_failure + 1;
                        torque(j,k) = 0;
                    end
                end
            end

            acceleration = ...
                (torque(:,k)+d-b.*rate(:,k))./inertia;

            rate(:,k+1) = rate(:,k) + acceleration*dt;
            angle(:,k+1) = angle(:,k) + rate(:,k+1)*dt;
        end

        err_deg = rad2deg(ref-angle);

        rw = t >= 4 & t <= 5.5;
        pw = t >= 6 & t <= 7.5;

        mse_roll = mean(err_deg(1,rw).^2);
        mse_pitch = mean(err_deg(2,pw).^2);

        peak_roll = max(abs(err_deg(1,rw)));
        peak_pitch = max(abs(err_deg(2,pw)));

        rms_torque_roll = sqrt(mean(torque(1,:).^2));
        rms_torque_pitch = sqrt(mean(torque(2,:).^2));

        results = [results;
            disturbance_strength, c, ...
            mse_roll, mse_pitch, ...
            peak_roll, peak_pitch, ...
            rms_torque_roll, rms_torque_pitch, ...
            solver_success, solver_failure];

    end
end

%% Results table
Results = array2table(results, ...
    'VariableNames', { ...
    'Disturbance_Nm','ControllerID', ...
    'RollMSE','PitchMSE', ...
    'RollPeakDeg','PitchPeakDeg', ...
    'RollRMSTorque','PitchRMSTorque', ...
    'MPCSuccesses','MPCFailures'});

Results.Controller = names(Results.ControllerID)';
Results = movevars(Results,'Controller','Before','ControllerID');

disp(Results);
fprintf('\nDisturbance sweep complete.\n');
