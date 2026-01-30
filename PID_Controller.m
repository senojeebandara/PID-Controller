%% DC Motor PID Control Simulation - Separate Graphs (Auto-Tuned for P, PI, PID)
clear; close all; clc;

%% 1) Motor Parameters
J = 0.02;    % Moment of inertia (kg.m^2)
b = 0.05;    % Damping coefficient (N.m.s)
K = 0.02;    % Motor torque constant (N.m/A)
R = 2;       % Armature resistance (ohm)
L = 1;       % Armature inductance (H)

% Transfer function from input voltage to angular position
num = K;
den = [J*L, (J*R + L*b), (b*R + K^2)];
motor_tf = tf(num, den);

disp('=== DC Motor Transfer Function ===');
motor_tf

%% 2) Auto-Tune P, PI, and PID Controllers
[pid_P, infoP]    = pidtune(motor_tf, 'P');
[pid_PI, infoPI]  = pidtune(motor_tf, 'PI');
[pid_PID, infoPID]= pidtune(motor_tf, 'PID');

fprintf('\n=== Auto-Tuned Controller Gains ===\n');
fprintf('P Controller:\n');
fprintf('  Kp = %.3f\n', pid_P.Kp);

fprintf('\nPI Controller:\n');
fprintf('  Kp = %.3f, Ki = %.3f\n', pid_PI.Kp, pid_PI.Ki);

fprintf('\nPID Controller:\n');
fprintf('  Kp = %.3f, Ki = %.3f, Kd = %.3f\n', pid_PID.Kp, pid_PID.Ki, pid_PID.Kd);

% Closed-loop systems (unity feedback)
cl_P   = feedback(pid_P   * motor_tf, 1);
cl_PI  = feedback(pid_PI  * motor_tf, 1);
cl_PID = feedback(pid_PID * motor_tf, 1);

%% 3) Step Response (0° → 90°)
t = 0:0.01:10;
[y_step, t_step] = step(90 * cl_PID, t);

% Performance metrics
S = stepinfo(y_step, t_step, 90);
fprintf('\n=== Step Response Performance (PID) ===\n');
fprintf('Rise Time: %.3f s\n', S.RiseTime);
fprintf('Overshoot: %.2f %%\n', S.Overshoot);
fprintf('Settling Time: %.3f s\n', S.SettlingTime);

% Plot 1: Step Response
figure('Name', 'Step Response (0° → 90°)', 'NumberTitle', 'off');
plot(t_step, y_step, 'b', 'LineWidth', 2);
hold on;
plot(t_step, 90*ones(size(t_step)), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (deg)');
title('PID Step Response (0° → 90°)');
legend('PID Response', 'Reference (90°)', 'Location', 'southeast');
grid on;
ylim([0 100]);

%% 4) Step Response with Disturbance
disturbance = zeros(size(t));
disturbance(t >= 2 & t <= 2.2) = -10;  % Small external torque disturbance

ref = 90 * ones(size(t));
[y_disturbed, ~] = lsim(cl_PID, ref + disturbance, t);

% Plot 2: Disturbance Response
figure('Name', 'Response with Disturbance', 'NumberTitle', 'off');
plot(t, ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(t, y_disturbed, 'b', 'LineWidth', 2, 'DisplayName', 'PID Response');
xline(2, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Disturbance at 2s');
xlabel('Time (s)');
ylabel('Position (deg)');
title('PID Response to Step Input with External Disturbance');
legend('Location', 'southeast');
grid on;
ylim([0 100]);

%% 5) Controller Comparison (P, PI, PID)
[yP, tP]    = step(90 * cl_P,  t);
[yPI, tPI]  = step(90 * cl_PI, t);
[yPID, tPID]= step(90 * cl_PID, t);

% Plot 3: Controller Comparison
figure('Name', 'Controller Comparison (P vs PI vs PID)', 'NumberTitle', 'off');
plot(tP, yP, 'r-', 'LineWidth', 2, 'DisplayName', 'P Controller');
hold on;
plot(tPI, yPI, 'g-', 'LineWidth', 2, 'DisplayName', 'PI Controller');
plot(tPID, yPID, 'b-', 'LineWidth', 2, 'DisplayName', 'PID Controller');
plot(t, 90*ones(size(t)), 'k--', 'LineWidth', 1, 'DisplayName', 'Reference (90°)');
xlabel('Time (s)');
ylabel('Position (deg)');
title('Comparison of P, PI, and PID Controllers');
legend('Location', 'southeast');
grid on;
xlim([0 10]);
ylim([0 100]);

%% 6) Performance Summary Table
fprintf('\n=== Performance Summary ===\n');
fprintf('Controller | Rise(s) | Overshoot(%%) | Settling(s)\n');
fprintf('-----------|----------|---------------|-------------\n');
S_P   = stepinfo(yP, tP, 90);
S_PI  = stepinfo(yPI, tPI, 90);
S_PID = stepinfo(yPID, tPID, 90);
fprintf('P    | %6.2f | %6.2f | %6.2f\n', S_P.RiseTime,  S_P.Overshoot,  S_P.SettlingTime);
fprintf('PI   | %6.2f | %6.2f | %6.2f\n', S_PI.RiseTime, S_PI.Overshoot, S_PI.SettlingTime);
fprintf('PID  | %6.2f | %6.2f | %6.2f\n', S_PID.RiseTime, S_PID.Overshoot, S_PID.SettlingTime);
