clear;clc;close all;
t_simu = 0:0.001:25; 
state_initial = [0.2; 0.15; 0; 0; 0; 0]; % [q1, q2, q1d, q2d, i_e1, i_e2]
q_final = [0; 0]; 

%% CASE 1: P - Low Kp
K1_case7 = [50, 0, 0];
K2_case7 = [30, 0, 0];  
[t7, state7] = ode45(@(t,y) state_equ_vector(t,y, K1_case7, K2_case7, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_7 = q_final(1) - state7(:,1); e2_7 = q_final(2) - state7(:,2);
e1_dot_7 = -state7(:,3); e2_dot_7 = -state7(:,4);
tau1_7 = K1_case7(1)*e1_7 + K1_case7(2)*e1_dot_7;
tau2_7 = K2_case7(1)*e2_7 + K2_case7(2)*e2_dot_7;
% Plotting
figure('Name', 'Case 1: P Low Kp - Torques'); sgtitle('Case 1: P Controller Low Kp - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t7, tau1_7, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t7, tau2_7, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 2: P - High Kp
K1_case8 = [1000, 0, 0];
K2_case8 = [800, 0, 0];  
[t8, state8] = ode45(@(t,y) state_equ_vector(t,y, K1_case8, K2_case8, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_8 = q_final(1) - state8(:,1); e2_8 = q_final(2) - state8(:,2);
e1_dot_8 = -state8(:,3); e2_dot_8 = -state8(:,4);
tau1_8 = K1_case8(1)*e1_8 + K1_case8(2)*e1_dot_8;
tau2_8 = K2_case8(1)*e2_8 + K2_case8(2)*e2_dot_8;
% Plotting
figure('Name', 'Case 2: P High Kp - Torques'); sgtitle('Case 2: P Controller High Kp - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t8, tau1_8, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t8, tau2_8, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 3: D - Low Kd
K1_case9 = [0, 2, 0];
K2_case9 = [0, 1, 0];  
[t9, state9] = ode45(@(t,y) state_equ_vector(t,y, K1_case9, K2_case9, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_9 = q_final(1) - state9(:,1); e2_9 = q_final(2) - state9(:,2);
e1_dot_9 = -state9(:,3); e2_dot_9 = -state9(:,4);
tau1_9 = K1_case9(1)*e1_9 + K1_case9(2)*e1_dot_9;
tau2_9 = K2_case9(1)*e2_9 + K2_case9(2)*e2_dot_9;
% Plotting
figure('Name', 'Case 3: D Low Kd - Torques'); sgtitle('Case 3: D Controller Low Kd - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t9, tau1_9, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t9, tau2_9, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 4: D - High Kd
K1_case10 = [0, 100, 0];
K2_case10 = [0, 90, 0];  
[t10, state10] = ode45(@(t,y) state_equ_vector(t,y, K1_case10, K2_case10, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_10 = q_final(1) - state10(:,1); e2_10 = q_final(2) - state10(:,2);
e1_dot_10 = -state10(:,3); e2_dot_10 = -state10(:,4);
tau1_10 = K1_case10(1)*e1_10 + K1_case10(2)*e1_dot_10;
tau2_10 = K2_case10(1)*e2_10 + K2_case10(2)*e2_dot_10;
% Plotting
figure('Name', 'Case 4: D High Kd - Torques'); sgtitle('Case 4: D Controller High Kd - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t10, tau1_10, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t10, tau2_10, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 5: PD Controller - Low Kd, Moderate Kp (Underdamped)
K1_case1 = [250, 1, 0];
K2_case1 = [250, 1, 0];  
[t1, state1] = ode45(@(t,y) state_equ_vector(t,y, K1_case1, K2_case1, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_1 = q_final(1) - state1(:,1); e2_1 = q_final(2) - state1(:,2);
e1_dot_1 = -state1(:,3); e2_dot_1 = -state1(:,4);
tau1_1 = K1_case1(1)*e1_1 + K1_case1(2)*e1_dot_1;
tau2_1 = K2_case1(1)*e2_1 + K2_case1(2)*e2_dot_1;
% Plotting
figure('Name', 'Case 5: PD Moderate Kp Low Kd - Torques'); sgtitle('Case 5: PD Controller Moderate Kp Low Kd - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t1, tau1_1, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t1, tau2_1, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 6: PD Controller - High Kp, High Kd (Critically Damped)
K1_case2 = [1500, 200, 0];
K2_case2 = [1500, 200, 0];
[t2, state2] = ode45(@(t,y) state_equ_vector(t,y, K1_case2, K2_case2, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_2 = q_final(1) - state2(:,1); e2_2 = q_final(2) - state2(:,2);
e1_dot_2 = -state2(:,3); e2_dot_2 = -state2(:,4);
tau1_2 = K1_case2(1)*e1_2 + K1_case2(2)*e1_dot_2;
tau2_2 = K2_case2(1)*e2_2 + K2_case2(2)*e2_dot_2;
% Plotting
figure('Name', 'Case 6: PD Critically Damped - Torques'); sgtitle('Case 6: PD Controller - Critically Damped - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t2, tau1_2, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t2, tau2_2, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 7: PI Controller - Low Ki (Slow Convergence)
K1_case3 = [1000, 0, 5];
K2_case3 = [800, 0, 3];
[t3, state3] = ode45(@(t,y) state_equ_vector(t, y, K1_case3, K2_case3, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_3 = q_final(1) - state3(:,1); e2_3 = q_final(2) - state3(:,2);
e1_dot_3 = -state3(:,3); e2_dot_3 = -state3(:,4);
int_e1_3 = state3(:,5); int_e2_3 = state3(:,6);
tau1_3 = K1_case3(1)*e1_3 + K1_case3(2)*e1_dot_3 + K1_case3(3)*int_e1_3;
tau2_3 = K2_case3(1)*e2_3 + K2_case3(2)*e2_dot_3 + K2_case3(3)*int_e2_3;
% Plotting
figure('Name', 'Case 7: PI Slow Convergence - Torques'); sgtitle('Case 7: PI Controller - Slow Convergence - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t3, tau1_3, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t3, tau2_3, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 8: PI Controller High Ki (Aggressive Integral)
K1_case4 = [1500, 0, 250];
K2_case4 = [1500, 0, 250];
[t4, state4] = ode45(@(t,y) state_equ_vector(t, y, K1_case4, K2_case4, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_4 = q_final(1) - state4(:,1); e2_4 = q_final(2) - state4(:,2);
e1_dot_4 = -state4(:,3); e2_dot_4 = -state4(:,4);
int_e1_4 = state4(:,5); int_e2_4 = state4(:,6);
tau1_4 = K1_case4(1)*e1_4 + K1_case4(2)*e1_dot_4 + K1_case4(3)*int_e1_4;
tau2_4 = K2_case4(1)*e2_4 + K2_case4(2)*e2_dot_4 + K2_case4(3)*int_e2_4;
% Plotting
figure('Name', 'Case 8: PI Aggressive - Torques'); sgtitle('Case 8: PI Controller - Aggressive Integral - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t4, tau1_4, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t4, tau2_4, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 9: PID Controller - Balanced Gains (Optimal Performance)
K1_case5 = [1500, 200, 250];
K2_case5 = [1500, 200, 250];
[t5, state5] = ode45(@(t,y) state_equ_vector(t, y, K1_case5, K2_case5, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_5 = q_final(1) - state5(:,1); e2_5 = q_final(2) - state5(:,2);
e1_dot_5 = -state5(:,3); e2_dot_5 = -state5(:,4);
int_e1_5 = state5(:,5); int_e2_5 = state5(:,6);
tau1_5 = K1_case5(1)*e1_5 + K1_case5(2)*e1_dot_5 + K1_case5(3)*int_e1_5;
tau2_5 = K2_case5(1)*e2_5 + K2_case5(2)*e2_dot_5 + K2_case5(3)*int_e2_5;
% Plotting
figure('Name', 'Case 9: PID Optimal - Torques'); sgtitle('Case 9: PID Controller - Balanced (Optimal) - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t5, tau1_5, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t5, tau2_5, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;

%% CASE 10: PID Controller-Damped
K1_case6 = [200, 1, 5];
K2_case6 = [200, 1, 5];
[t6, state6] = ode45(@(t,y) state_equ_vector(t, y, K1_case6, K2_case6, q_final,state_initial), t_simu, state_initial);
% Torque Calculation
e1_6 = q_final(1) - state6(:,1); e2_6 = q_final(2) - state6(:,2);
e1_dot_6 = -state6(:,3); e2_dot_6 = -state6(:,4);
int_e1_6 = state6(:,5); int_e2_6 = state6(:,6);
tau1_6 = K1_case6(1)*e1_6 + K1_case6(2)*e1_dot_6 + K1_case6(3)*int_e1_6;
tau2_6 = K2_case6(1)*e2_6 + K2_case6(2)*e2_dot_6 + K2_case6(3)*int_e2_6;
% Plotting
figure('Name', 'Case 10: PID Damped - Torques'); sgtitle('Case 10: PID Controller - Damped - Torques', 'FontSize', 14, 'FontWeight', 'bold');
subplot(2, 1, 1); plot(t6, tau1_6, 'm-', 'LineWidth', 1.5); title('Joint 1 Torque'); ylabel('Torque (Nm)'); grid on;
subplot(2, 1, 2); plot(t6, tau2_6, 'c-', 'LineWidth', 1.5); title('Joint 2 Torque'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;