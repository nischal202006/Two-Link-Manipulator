%% Mini Project: Control of a 2-Link Manipulator
% This script simulates and analyzes P, D, PD, PI, and PID controllers
% using the user-provided 'state_equ_vector' dynamics function.
clear;
clc;
close all;

%% 1. System and Simulation Parameters
% --- Simulation Setup ---
t_simu = 0:0.001:10; % Simulation time from 0 to 10 seconds (shortened for faster runs)

% --- Initial and Desired States ---
% Initial state: [q1, q2, q1_dot, q2_dot, integral_e1, integral_e2]
state_initial = [0.2; 0.15; 0; 0; 0; 0]; % rad, rad/s
q_final = [0; 0]; % Desired final joint angles [rad]
q_initial = state_initial(1:2); % For performance metrics calculation

%% 2. Run Simulations for Each Controller Case
fprintf('Running simulations with your state_equ_vector function...\n');

% CASE 1: P Controller (High Kp)
K1_P = [1500, 0, 0];
K2_P = [1500, 0, 0];
[t_p, state_p] = ode45(@(t,y) state_equ_vector(t, y, K1_P, K2_P, q_final), t_simu, state_initial);

% CASE 2: D Controller (High Kd)
K1_D = [0, 200, 0];
K2_D = [0, 200, 0];
[t_d, state_d] = ode45(@(t,y) state_equ_vector(t, y, K1_D, K2_D, q_final), t_simu, state_initial);

% CASE 3: PD Controller (Critically Damped)
K1_PD = [1500, 200, 0];
K2_PD = [1500, 200, 0];
[t_pd, state_pd] = ode45(@(t,y) state_equ_vector(t, y, K1_PD, K2_PD, q_final), t_simu, state_initial);

% CASE 4: PI Controller (High Ki)
K1_PI = [1500, 0, 250];
K2_PI = [1500, 0, 250];
[t_pi, state_pi] = ode45(@(t,y) state_equ_vector(t, y, K1_PI, K2_PI, q_final), t_simu, state_initial);

% CASE 5: PID Controller (Optimal)
K1_PID = [1500, 200, 250];
K2_PID = [1500, 200, 250];
[t_pid, state_pid] = ode45(@(t,y) state_equ_vector(t, y, K1_PID, K2_PID, q_final), t_simu, state_initial);

fprintf('Simulations complete.\n');

%% 3. Plotting the Results
% --- Plot for P Controller ---
figure('Name', 'P Controller (High Kp)');
sgtitle('P Controller (High Kp)', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2,1,1); plot(t_p, state_p(:,1), 'b', 'LineWidth', 1.5); hold on; yline(q_final(1), 'r--'); title(sprintf('Joint 1: Kp=%.0f', K1_P(1))); ylabel('Angle (rad)'); grid on; legend('q_1', 'Desired');
subplot(2,1,2); plot(t_p, state_p(:,2), 'g', 'LineWidth', 1.5); hold on; yline(q_final(2), 'r--'); title(sprintf('Joint 2: Kp=%.0f', K2_P(1))); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('q_2', 'Desired');

% --- Plot for D Controller ---
figure('Name', 'D Controller (High Kd)');
sgtitle('D Controller (High Kd)', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2,1,1); plot(t_d, state_d(:,1), 'b', 'LineWidth', 1.5); hold on; yline(q_final(1), 'r--'); title(sprintf('Joint 1: Kd=%.0f', K1_D(2))); ylabel('Angle (rad)'); grid on; legend('q_1', 'Desired');
subplot(2,1,2); plot(t_d, state_d(:,2), 'g', 'LineWidth', 1.5); hold on; yline(q_final(2), 'r--'); title(sprintf('Joint 2: Kd=%.0f', K2_D(2))); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('q_2', 'Desired');

% --- Plot for PD Controller ---
figure('Name', 'PD Controller');
sgtitle('PD Controller (Critically Damped)', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2,1,1); plot(t_pd, state_pd(:,1), 'b', 'LineWidth', 1.5); hold on; yline(q_final(1), 'r--'); title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f', K1_PD(1), K1_PD(2))); ylabel('Angle (rad)'); grid on; legend('q_1', 'Desired');
subplot(2,1,2); plot(t_pd, state_pd(:,2), 'g', 'LineWidth', 1.5); hold on; yline(q_final(2), 'r--'); title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f', K2_PD(1), K2_PD(2))); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('q_2', 'Desired');

% --- Plot for PI Controller ---
figure('Name', 'PI Controller');
sgtitle('PI Controller (High Ki)', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2,1,1); plot(t_pi, state_pi(:,1), 'b', 'LineWidth', 1.5); hold on; yline(q_final(1), 'r--'); title(sprintf('Joint 1: Kp=%.0f, Ki=%.0f', K1_PI(1), K1_PI(3))); ylabel('Angle (rad)'); grid on; legend('q_1', 'Desired');
subplot(2,1,2); plot(t_pi, state_pi(:,2), 'g', 'LineWidth', 1.5); hold on; yline(q_final(2), 'r--'); title(sprintf('Joint 2: Kp=%.0f, Ki=%.0f', K2_PI(1), K2_PI(3))); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('q_2', 'Desired');

% --- Plot for PID Controller ---
figure('Name', 'PID Controller');
sgtitle('PID Controller (Optimal)', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2,1,1); plot(t_pid, state_pid(:,1), 'b', 'LineWidth', 1.5); hold on; yline(q_final(1), 'r--'); title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_PID(1), K1_PID(2), K1_PID(3))); ylabel('Angle (rad)'); grid on; legend('q_1', 'Desired');
subplot(2,1,2); plot(t_pid, state_pid(:,2), 'g', 'LineWidth', 1.5); hold on; yline(q_final(2), 'r--'); title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_PID(1), K2_PID(2), K2_PID(3))); xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('q_2', 'Desired');

%% 4. Performance Metrics Calculation and Display
fprintf('Calculating performance metrics...\n');

results = struct();
% --- Calculate metrics for each controller ---
results.P   = calculate_metrics(t_p,   state_p,   K1_P,   K2_P,   q_final, q_initial);
results.D   = calculate_metrics(t_d,   state_d,   K1_D,   K2_D,   q_final, q_initial);
results.PD  = calculate_metrics(t_pd,  state_pd,  K1_PD,  K2_PD,  q_final, q_initial);
results.PI  = calculate_metrics(t_pi,  state_pi,  K1_PI,  K2_PI,  q_final, q_initial);
results.PID = calculate_metrics(t_pid, state_pid, K1_PID, K2_PID, q_final, q_initial);

% --- Display the results in a formatted table ---
fprintf('\n\n--- Performance Metrics Comparison ---\n');
fprintf('%-30s %10s %10s %10s %10s %10s\n', 'Metric', 'P', 'D', 'PD', 'PI', 'PID');
fprintf(repmat('-', 1, 85));
fprintf('\n');
print_row = @(metric_name, field) fprintf('%-30s %10.3f %10.3f %10.3f %10.3f %10.3f\n', ...
    metric_name, results.P.(field), results.D.(field), results.PD.(field), results.PI.(field), results.PID.(field));

print_row('Settling Time (Joint 1) [s]', 'settling_time1');
print_row('Settling Time (Joint 2) [s]', 'settling_time2');
print_row('Overshoot (Joint 1) [%]', 'overshoot1');
print_row('Overshoot (Joint 2) [%]', 'overshoot2');
print_row('Steady-State Error (Joint 1) [rad]', 'ss_error1');
print_row('Steady-State Error (Joint 2) [rad]', 'ss_error2');
print_row('Max Torque (Joint 1) [Nm]', 'max_torque1');
print_row('Max Torque (Joint 2) [Nm]', 'max_torque2');
fprintf(repmat('-', 1, 85));
fprintf('\n');

%% 5. FUNCTION DEFINITIONS
% This section contains all the functions needed for the script to run.
% You can place these at the end of your script file.

function state_equ = state_equ_vector(t, state_vector, K1, K2, q_final)
    % This is the dynamics and controller function you provided.
    q1 = state_vector(1);
    q2 = state_vector(2);
    q1_dot = state_vector(3);
    q2_dot = state_vector(4);
    int_e1 = state_vector(5);  
    int_e2 = state_vector(6);  
    
    q_dot = [q1_dot; q2_dot];
    Kp1 = K1(1); 
    Kd1 = K1(2); 
    Ki1 = K1(3);
    Kp2 = K2(1); 
    Kd2 = K2(2); 
    Ki2 = K2(3);

    e1 = q_final(1) - q1;
    e2 = q_final(2) - q2;
    e1_dot = -q1_dot;  
    e2_dot = -q2_dot;  

    tau1 = Kp1*e1 + Kd1*e1_dot + Ki1*int_e1;
    tau2 = Kp2*e2 + Kd2*e2_dot + Ki2*int_e2;
    TAU = [tau1; tau2];

    m1 = 5;    
    m2 = 3;    
    l1 = 0.25; 
    l2 = 0.15; 
    g = 9.81;  

    M11 = (m1 + m2)*(l1^2) + m2*l2*(l2 + 2*l1*cos(q2));
    M12 = m2*l2*(l2 + l1*cos(q2));
    M21 = m2*l2*(l2 + l1*cos(q2));
    M22 = m2*l2*l2;
    M = [M11, M12; M21, M22];

    C11 = -m2*l1*l2*sin(q2)*q2_dot;
    C12 = -m2*l1*l2*sin(q2)*(q2_dot + q1_dot);
    C21 = 0; % Note: As per your provided function
    C22 = m2*l1*l2*sin(q2)*q1_dot;
    C = [C11, C12; C21, C22];
    
    G = [m1*l1*g*cos(q1) + m2*g*(l2*cos(q1 + q2) + l1*cos(q1)); 
         m2*g*l2*cos(q1 + q2)];
    
    q_dotdot = M \ (TAU - C*q_dot - G);
    
    state_equ = [q_dot;      
                 q_dotdot;   
                 e1;        
                 e2];        
end

function metrics = calculate_metrics(t, state, K1, K2, q_final, q_initial)
    % This helper function computes performance metrics from simulation data.
    q1=state(:,1); q2=state(:,2); q1_dot=state(:,3); q2_dot=state(:,4);
    int_e1=state(:,5); int_e2=state(:,6);
    
    % --- Settling Time (MODIFIED LOGIC) ---
    % First, determine the actual steady-state value by averaging the last 5% of data points.
    % This handles cases with non-zero steady-state error (like P-control).
    num_ss_points = max(10, floor(0.05 * length(t)));
    ss_val1 = mean(q1(end-num_ss_points+1:end));
    ss_val2 = mean(q2(end-num_ss_points+1:end));
    
    % Define tolerance as 2% of the total response amplitude (from initial to final value).
    % This is more robust than using a percentage of the initial or final value alone.
    tol1 = 0.02 * abs(ss_val1 - q_initial(1));
    if tol1 == 0; tol1 = 0.001; end % Avoid tolerance of 0 if it settles perfectly at the start
    tol2 = 0.02 * abs(ss_val2 - q_initial(2));
    if tol2 == 0; tol2 = 0.001; end

    % Find the last time the system was outside the tolerance band of its *actual* steady state
    outside1 = abs(q1 - ss_val1) > tol1;
    last_idx1 = find(outside1, 1, 'last');
    
    outside2 = abs(q2 - ss_val2) > tol2;
    last_idx2 = find(outside2, 1, 'last');

    metrics.settling_time1 = ifthen(isempty(last_idx1), 0, t(last_idx1));
    metrics.settling_time2 = ifthen(isempty(last_idx2), 0, t(last_idx2));
    
    % If the system never settles, report NaN.
    if metrics.settling_time1 >= t(end) * 0.99; metrics.settling_time1 = NaN; end
    if metrics.settling_time2 >= t(end) * 0.99; metrics.settling_time2 = NaN; end
    
    % --- Percentage Overshoot ---
    % Calculated relative to the initial and desired final values.
    response_span1 = abs(q_initial(1) - q_final(1));
    response_span2 = abs(q_initial(2) - q_final(2));
    
    max_overshoot1 = max(0, q_final(1) - min(q1)); % Finds max deviation in the overshoot direction
    max_overshoot2 = max(0, q_final(2) - min(q2));
    
    metrics.overshoot1 = ifthen(response_span1 > 1e-6, (max_overshoot1 / response_span1) * 100, 0);
    metrics.overshoot2 = ifthen(response_span2 > 1e-6, (max_overshoot2 / response_span2) * 100, 0);

    % --- Steady-State Error ---
    % This is the difference between the desired final value and the actual final value.
    metrics.ss_error1 = abs(q_final(1) - ss_val1);
    metrics.ss_error2 = abs(q_final(2) - ss_val2);
    
    % --- Max Torque ---
    e1 = q_final(1) - q1; 
    e2 = q_final(2) - q2; 
    e1_dot = -q1_dot; 
    e2_dot = -q2_dot;
    tau1 = K1(1)*e1 + K1(2)*e1_dot + K1(3)*int_e1;
    tau2 = K2(1)*e2 + K2(2)*e2_dot + K2(3)*int_e2;
    metrics.max_torque1 = max(abs(tau1));
    metrics.max_torque2 = max(abs(tau2));
end

function out = ifthen(cond, true_val, false_val)
    % Helper for one-line if/else statements
    if cond; out = true_val; else; out = false_val; end
end