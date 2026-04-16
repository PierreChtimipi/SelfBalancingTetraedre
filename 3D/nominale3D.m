%% Expérience 1 : Stabilisation Nominale (Angle faible 5°)
clear all; close all; clc;

% --- Paramètres Physiques ---
g = 9.81; 
mb = 0.5; % 500g masse
lb = 0.075; % bras de levier
Ieq = 0.06;  % inertie (m*d²)
Cb = 0.001; % résistance de lair

% --- Configuration ---
dt = 0.005; t_final = 3; n_steps = t_final/dt;
theta = zeros(1, n_steps); theta_dot = zeros(1, n_steps); time = zeros(1, n_steps);
theta(1) = deg2rad(5); % Petite perturbation de 5°

% --- PID Nominal --- 
Kp = 7.0; %force de rappel
Kd = 1.8; % amortisseur
Ki = 0.3; % corrige les erreurs 
integral_error = 0;

for k = 1:n_steps-1
    error = 0 - theta(k);
    integral_error = integral_error + error * dt;
    
    % Loi de commande PID
    Tm_pid = Kp*error + Ki*integral_error - Kd*theta_dot(k);
    Tm = max(min(Tm_pid, 0.10), -0.10); % Saturation EMAX 
    
    % Dynamique
    theta_ddot = (-mb*g*lb * sin(theta(k)) + Tm - Cb*theta_dot(k)) / Ieq;
    
    % Methode d'Euler prédictive
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1) = theta(k) + theta_dot(k) * dt;
    time(k+1) = time(k) + dt;
end

figure('Color', 'w'); plot(time, rad2deg(theta), 'b', 'LineWidth', 2);
grid on; line([0 t_final], [0 0], 'Color', 'k', 'LineStyle', '--');
title('Exp 1 : Stabilisation Nominale (Perturbation 5°)');
xlabel('Temps (s)'); ylabel('Angle (°)'); ylim([-2 6]);