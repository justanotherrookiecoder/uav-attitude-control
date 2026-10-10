
clear; clc; close all;

% Simulation settings
dt = 0.001;
T = 10;
t = 0:dt:T;
N = length(t);

% Nominal UAV parameters
I0 = [0.020; 0.025];   % Nominal roll/pitch inertia
b  = [0.020; 0.020];   % Angular damping

% Fixed PID gains (same as baseline)
Kp = [0.8; 0.8];
Ki = [0.05; 0.05];
Kd = [0.15; 0.15];

% Desired angles: roll 10 deg, pitch -5 deg
ref = deg2rad([10; -5]);

% Data arrays
angle = zeros(2,N);
rate = zeros(2,N);
torque = zeros(2,N);
disturbance = zeros(2,N);
inertia = zeros(2,N);
integral = zeros(2,1);

for k = 1:N-1

    % Illustrative inertia variation
    % Roll inertia increases by 30% from 4 to 6 seconds
    % Pitch inertia increases by 20% from 6 to 8 seconds
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

    % Fixed-gain PID
    error = ref - angle(:,k);
    integral = integral + error*dt;
    integral = max(min(integral,2),-2);

    u = Kp.*error + Ki.*integral - Kd.*rate(:,k);
    torque(:,k) = max(min(u,0.5),-0.5);

    % Variable-inertia attitude dynamics
    acceleration = (torque(:,k) + disturbance(:,k) ...
        - b.*rate(:,k))./inertia(:,k);

    % Numerical integration
    rate(:,k+1) = rate(:,k) + acceleration*dt;
    angle(:,k+1) = angle(:,k) + rate(:,k+1)*dt;
end

inertia(:,N) = inertia(:,N-1);
torque(:,N) = torque(:,N-1);

% Plot attitude response
figure;
subplot(2,1,1);
plot(t,rad2deg(angle(1,:)),'b','LineWidth',1.4);
hold on; yline(10,'--k');
grid on; ylabel('Roll (deg)');
legend('Actual','Reference');

subplot(2,1,2);
plot(t,rad2deg(angle(2,:)),'r','LineWidth',1.4);
hold on; yline(-5,'--k');
grid on; ylabel('Pitch (deg)');
xlabel('Time (s)');
legend('Actual','Reference');

% Plot inertia variation
figure;
plot(t,inertia(1,:),'b',t,inertia(2,:),'r', ...
    'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Moment of inertia (kg m^2)');
legend('Roll inertia','Pitch inertia');
title('Illustrative Time-Varying Inertia');
fprintf('Final roll: %.2f deg\n',rad2deg(angle(1,end)));
fprintf('Final pitch: %.2f deg\n',rad2deg(angle(2,end)));
fprintf('Final roll inertia: %.4f kg m^2\n',inertia(1,end));
fprintf('Final pitch inertia: %.4f kg m^2\n',inertia(2,end));

% Tracking error in degrees
error_deg = rad2deg(ref - angle);

% Disturbance and recovery windows
roll_window = t >= 4 & t <= 5.5;
pitch_window = t >= 6 & t <= 7.5;

% Mean squared error during disturbance and recovery
roll_mse = mean(error_deg(1,roll_window).^2);
pitch_mse = mean(error_deg(2,pitch_window).^2);

% Maximum absolute tracking error in each window
roll_peak = max(abs(error_deg(1,roll_window)));
pitch_peak = max(abs(error_deg(2,pitch_window)));

fprintf('\n--- Variable-Inertia PID Results ---\n');
fprintf('Roll disturbance MSE: %.4f deg^2\n',roll_mse);
fprintf('Pitch disturbance MSE: %.4f deg^2\n',pitch_mse);
fprintf('Roll peak error: %.4f deg\n',roll_peak);
fprintf('Pitch peak error: %.4f deg\n',pitch_peak);
