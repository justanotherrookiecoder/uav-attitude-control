
clear; clc;

%% Simulation settings
dt = 0.001;
T = 10;
t = 0:dt:T;
N = length(t);

%% Nominal UAV parameters
I0 = [0.020; 0.025];
b  = [0.020; 0.020];

ref = deg2rad([10; -5]);

%% MPC settings
Np = 100;                  % Prediction horizon
Nc = 10;                   % Control horizon
Q = diag([100, 2]);       % Angle and angular-rate weights
R = 0.5;                  % Control effort weight
u_max = 0.5;              % Maximum torque (N m)

options = optimoptions('quadprog','Display','off');

%% Data arrays
angle = zeros(2,N);
rate = zeros(2,N);
torque = zeros(2,N);
disturbance = zeros(2,N);
inertia = zeros(2,N);

solver_success = 0;
solver_failure = 0;

%% Main simulation loop
for k = 1:N-1

    % Variable inertia
    inertia(:,k) = I0;

    if t(k) >= 4 && t(k) < 6
        inertia(1,k) = 1.30*I0(1);
    end

    if t(k) >= 6 && t(k) < 8
        inertia(2,k) = 1.20*I0(2);
    end

    % External torque disturbances
    if t(k) >= 4 && t(k) < 4.5
        disturbance(1,k) = 0.08;
    end

    if t(k) >= 6 && t(k) < 6.4
        disturbance(2,k) = -0.08;
    end

    % Solve MPC independently for roll and pitch
    for j = 1:2

        I = inertia(j,k);

        % Discrete-time prediction model
        A = [1, dt;
             0, 1-b(j)*dt/I];

        B = [0; dt/I];

        xj = [angle(j,k); rate(j,k)];
        rj = ref(j);

        % Prediction matrices
        F = zeros(2*Np,2);
        G = zeros(2*Np,Nc);

        for p = 1:Np
            rows = 2*p-1:2*p;
            F(rows,:) = A^p;

            for m = 1:min(p,Nc)
                G(rows,m) = A^(p-m)*B;
            end
        end

        % Reference trajectory
        Xref = repmat([rj; 0],Np,1);

        % Predicted effect of current disturbance
        Ed = [0; dt/I]*disturbance(j,k);
        D = zeros(2*Np,1);

        for p = 1:Np
            rows = 2*p-1:2*p;

            for q = 0:p-1
                D(rows) = D(rows) + A^q*Ed;
            end
        end

        % Quadratic objective
        Qbar = kron(eye(Np),Q);
        Rbar = R*eye(Nc);

        H = 2*(G'*Qbar*G + Rbar);
        H = (H+H')/2;

        f = 2*G'*Qbar*(F*xj + D - Xref);

        % Torque constraints
        lb = -u_max*ones(Nc,1);
        ub =  u_max*ones(Nc,1);

        % Optimize control sequence
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

    % Simulated UAV attitude dynamics
    acceleration = (torque(:,k) + disturbance(:,k) ...
        - b.*rate(:,k))./inertia(:,k);

    % Numerical integration
    rate(:,k+1) = rate(:,k) + acceleration*dt;
    angle(:,k+1) = angle(:,k) + rate(:,k+1)*dt;
end

inertia(:,N) = inertia(:,N-1);
torque(:,N) = torque(:,N-1);

%% Performance metrics
error_deg = rad2deg(ref-angle);

roll_window = t >= 4 & t <= 5.5;
pitch_window = t >= 6 & t <= 7.5;

roll_mse = mean(error_deg(1,roll_window).^2);
pitch_mse = mean(error_deg(2,pitch_window).^2);

roll_peak = max(abs(error_deg(1,roll_window)));
pitch_peak = max(abs(error_deg(2,pitch_window)));

%% Print results
fprintf('\n--- MPC Simulation Results ---\n');

fprintf('Roll disturbance MSE: %.4f deg^2\n',roll_mse);
fprintf('Pitch disturbance MSE: %.4f deg^2\n',pitch_mse);

fprintf('Roll peak error: %.4f deg\n',roll_peak);
fprintf('Pitch peak error: %.4f deg\n',pitch_peak);

fprintf('Final roll: %.2f deg\n',rad2deg(angle(1,end)));
fprintf('Final pitch: %.2f deg\n',rad2deg(angle(2,end)));

fprintf('\n--- Solver Diagnostics ---\n');
fprintf('Successful MPC solves: %d\n',solver_success);
fprintf('Failed MPC solves: %d\n',solver_failure);
fprintf('Total MPC solves: %d\n', ...
    solver_success+solver_failure);
fprintf('Expected solves: %d\n',2*(N-1));
