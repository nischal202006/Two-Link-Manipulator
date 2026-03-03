clear;clc;close all;

t_simu = 0:0.001:25; 
state_initial = [0.2; 0.15; 0; 0; 0; 0]; % [q1, q2, q1d, q2d, i_e1, i_e2]
q_final = [0; 0]; 

%% CASE 1: P - Low Kp
K1_case7 = [50, 0, 0];
K2_case7 = [30, 0, 0];  

[t7, state7] = ode45(@(t,y) state_equ_vector(t,y, K1_case7, K2_case7, q_final,state_initial), t_simu, state_initial);

e1_7 = q_final(1) - state7(:,1); e2_7 = q_final(2) - state7(:,2);
e1_dot_7 = -state7(:,3); e2_dot_7 = -state7(:,4);
tau1_7 = K1_case7(1)*e1_7 + K1_case7(2)*e1_dot_7;
tau2_7 = K2_case7(1)*e2_7 + K2_case7(2)*e2_dot_7;

figure('Name', 'Case 1: P Low Kp');
sgtitle('Case 1: P Controller Low Kp', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t7, state7(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case7));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t7, state7(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case7));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 1: P Low Kp - Torques'); sgtitle('Case 1: P Controller Low Kp - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t7, tau1_7, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t7, tau2_7, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 2: P - High Kp
K1_case8 = [1000, 0, 0];
K2_case8 = [800, 0, 0];  

[t8, state8] = ode45(@(t,y) state_equ_vector(t,y, K1_case8, K2_case8, q_final,state_initial), t_simu, state_initial);

e1_8 = q_final(1) - state8(:,1); e2_8 = q_final(2) - state8(:,2);
e1_dot_8 = -state8(:,3); e2_dot_8 = -state8(:,4);
tau1_8 = K1_case8(1)*e1_8 + K1_case8(2)*e1_dot_8;
tau2_8 = K2_case8(1)*e2_8 + K2_case8(2)*e2_dot_8;

figure('Name', 'Case 2: P High Kp');
sgtitle('Case 2: P Controller High Kp', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t8, state8(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case8));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t8, state8(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case8));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 2: P High Kp - Torques'); sgtitle('Case 2: P Controller High Kp - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t8, tau1_8, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t8, tau2_8, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 3: PD Controller - Low Kd, Moderate Kp (Underdamped)
K1_case1 = [250, 1, 0];
K2_case1 = [250, 1, 0];  

[t1, state1] = ode45(@(t,y) state_equ_vector(t,y, K1_case1, K2_case1, q_final,state_initial), t_simu, state_initial);

e1_1 = q_final(1) - state1(:,1); e2_1 = q_final(2) - state1(:,2);
e1_dot_1 = -state1(:,3); e2_dot_1 = -state1(:,4);
tau1_1 = K1_case1(1)*e1_1 + K1_case1(2)*e1_dot_1;
tau2_1 = K2_case1(1)*e2_1 + K2_case1(2)*e2_dot_1;

figure('Name', 'Case 3: PD Moderate Kp Low Kd');
sgtitle('Case 3: PD Controller Moderate Kp Low Kd', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t1, state1(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case1));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t1, state1(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case1));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 5: PD Moderate Kp Low Kd - Torques'); sgtitle('Case 5: PD Controller Moderate Kp Low Kd - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t1, tau1_1, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t1, tau2_1, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 4: PD Controller - High Kp, High Kd (Critically Damped)
K1_case2 = [1200, 50, 0];
K2_case2 = [900, 40, 0];

[t2, state2] = ode45(@(t,y) state_equ_vector(t,y, K1_case2, K2_case2, q_final,state_initial), t_simu, state_initial);

e1_2 = q_final(1) - state2(:,1); e2_2 = q_final(2) - state2(:,2);
e1_dot_2 = -state2(:,3); e2_dot_2 = -state2(:,4);
tau1_2 = K1_case2(1)*e1_2 + K1_case2(2)*e1_dot_2;
tau2_2 = K2_case2(1)*e2_2 + K2_case2(2)*e2_dot_2;

figure('Name', 'Case 4: PD Critically Damped(High Kp, High Kd)');
sgtitle('Case 4: PD Controller - Critically Damped (High Kp, High Kd)', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t2, state2(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case2));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t2, state2(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case2));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 4: PD Critically Damped - Torques'); sgtitle('Case 6: PD Controller - Critically Damped - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t2, tau1_2, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t2, tau2_2, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 5: PI Controller - Low Ki (Slow Convergence)
K1_case3 = [1000, 0, 5]; % Kd1=0
K2_case3 = [800, 0, 3];  % Kd2=0

[t3, state3] = ode45(@(t,y) state_equ_vector(t, y, K1_case3, K2_case3, q_final,state_initial), t_simu, state_initial);

e1_3 = q_final(1) - state3(:,1); e2_3 = q_final(2) - state3(:,2);
e1_dot_3 = -state3(:,3); e2_dot_3 = -state3(:,4);
int_e1_3 = state3(:,5); int_e2_3 = state3(:,6);
tau1_3 = K1_case3(1)*e1_3 + K1_case3(2)*e1_dot_3 + K1_case3(3)*int_e1_3;
tau2_3 = K2_case3(1)*e2_3 + K2_case3(2)*e2_dot_3 + K2_case3(3)*int_e2_3;

figure('Name', 'Case 5: PI Slow Convergence(Low Ki)');
sgtitle('Case 5: PI Controller - Slow Convergence (Low Ki)', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t3, state3(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case3));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t3, state3(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case3));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 5: PI Slow Convergence - Torques'); sgtitle('Case 7: PI Controller - Slow Convergence - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t3, tau1_3, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t3, tau2_3, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 6: PI Controller High Ki (Aggressive Integral)
K1_case4 = [1000, 0, 50];
K2_case4 = [800, 0, 40];

[t4, state4] = ode45(@(t,y) state_equ_vector(t, y, K1_case4, K2_case4, q_final,state_initial), t_simu, state_initial);

e1_4 = q_final(1) - state4(:,1); e2_4 = q_final(2) - state4(:,2);
e1_dot_4 = -state4(:,3); e2_dot_4 = -state4(:,4);
int_e1_4 = state4(:,5); int_e2_4 = state4(:,6);
tau1_4 = K1_case4(1)*e1_4 + K1_case4(2)*e1_dot_4 + K1_case4(3)*int_e1_4;
tau2_4 = K2_case4(1)*e2_4 + K2_case4(2)*e2_dot_4 + K2_case4(3)*int_e2_4;

figure('Name', 'Case 6: PI Aggressive(High Ki)');
sgtitle('Case 6: PI Controller - Aggressive Integral (High Ki)', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t4, state4(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case4));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t4, state4(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case4));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 6: PI Aggressive - Torques'); sgtitle('Case 8: PI Controller - Aggressive Integral - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t4, tau1_4, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t4, tau2_4, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;


%% CASE 7: PID Controller - Balanced Gains (Optimal Performance)
K1_case5 = [1500, 200, 250];
K2_case5 = [1500, 200, 250];

[t5, state5] = ode45(@(t,y) state_equ_vector(t, y, K1_case5, K2_case5, q_final,state_initial), t_simu, state_initial);

e1_5 = q_final(1) - state5(:,1); e2_5 = q_final(2) - state5(:,2);
e1_dot_5 = -state5(:,3); e2_dot_5 = -state5(:,4);
int_e1_5 = state5(:,5); int_e2_5 = state5(:,6);
tau1_5 = K1_case5(1)*e1_5 + K1_case5(2)*e1_dot_5 + K1_case5(3)*int_e1_5;
tau2_5 = K2_case5(1)*e2_5 + K2_case5(2)*e2_dot_5 + K2_case5(3)*int_e2_5;

figure('Name', 'Case 7: PID Optimal');
sgtitle('Case 7: PID Controller - Balanced (Optimal) Performance', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t5, state5(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case5));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t5, state5(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case5));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 8: PID Optimal - Torques'); sgtitle('Case 9: PID Controller - Balanced (Optimal) - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t5, tau1_5, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t5, tau2_5, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 8: PID Controller-Damped
K1_case6 = [20,2, 5];
K2_case6 = [20,2, 5];

[t6, state6] = ode45(@(t,y) state_equ_vector(t, y, K1_case6, K2_case6, q_final,state_initial), t_simu, state_initial);

e1_6 = q_final(1) - state6(:,1); e2_6 = q_final(2) - state6(:,2);
e1_dot_6 = -state6(:,3); e2_dot_6 = -state6(:,4);
int_e1_6 = state6(:,5); int_e2_6 = state6(:,6);
tau1_6 = K1_case6(1)*e1_6 + K1_case6(2)*e1_dot_6 + K1_case6(3)*int_e1_6;
tau2_6 = K2_case6(1)*e2_6 + K2_case6(2)*e2_dot_6 + K2_case6(3)*int_e2_6;

figure('Name', 'Case 8: PID Damped');
sgtitle('Case 8: PID Controller -Damped', 'FontSize', 14, 'FontWeight', 'bold');
% Plot for Joint 1
subplot(2, 1, 1);
plot(t6, state6(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(1) q_final(1)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 1: Kp=%.0f, Kd=%.0f, Ki=%.0f', K1_case6));
ylabel('Angle (rad)'); grid on; legend('Actual q_1', 'Desired q_1', 'Location', 'best');
% Plot for Joint 2
subplot(2, 1, 2);
plot(t6, state6(:,2), 'g-', 'LineWidth', 1.5); hold on;
plot([0 25], [q_final(2) q_final(2)], 'r--', 'LineWidth', 1.5);
title(sprintf('Joint 2: Kp=%.0f, Kd=%.0f, Ki=%.0f', K2_case6));
xlabel('Time (s)'); ylabel('Angle (rad)'); grid on; legend('Actual q_2', 'Desired q_2', 'Location', 'best');

figure('Name', 'Case 9: PID Damped - Torques'); sgtitle('Case 10: PID Controller - Damped - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t6, tau1_6, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t6, tau2_6, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;