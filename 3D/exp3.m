%% Expérience 3 : Géométrie de Reuleaux (Instabilité Géométrique)
clear all; close all; clc;

% --- Paramètres ---
g = 9.81; mb = 0.5; L = 0.075; Ieq = 0.06; Cb = 0.001;

dt = 0.005; t_final = 3; n_steps = t_final/dt;
theta = zeros(1, n_steps); theta_dot = zeros(1, n_steps); time = zeros(1, n_steps);
theta(1) = deg2rad(15); % On teste sur 15° pour voir l'effet de la courbe

for k = 1:n_steps-1
    % Le bras de levier change car le robot "roule" sur son arête courbe
    lb_variable = L * cos(theta(k) * 0.5); 
    
    % Commande PID
    Kp = 7; Kd = 1.8; % On booste le Kd pour freiner
    error = 0 - theta(k);
    Tm_pid = Kp*error - Kd*theta_dot(k); 
    Tm = max(min(Tm_pid, 0.10), -0.10);
    
    % Dynamique avec levier variable
    theta_ddot = (-mb*g*lb_variable * sin(theta(k)) + Tm - Cb*theta_dot(k)) / Ieq;
    
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1) = theta(k) + theta_dot(k) * dt;
    time(k+1) = time(k) + dt;
end

figure('Color', 'w'); plot(time, rad2deg(theta), 'g', 'LineWidth', 2);
grid on; line([0 t_final], [0 0], 'Color', 'k', 'LineStyle', '--');
title('Exp 3 : Stabilisation sur face courbe de Reuleaux');
xlabel('Temps (s)'); ylabel('Angle (°)'); ylim([-5 20]);