%% Expérience : Stabilisation 2D - Face de Tétraèdre de Reuleaux
clear all; close all; clc;

% --- Paramètres Physiques (Modèle Face 2D) ---
g = 9.81; 
m_face = 0.250;     % Masse d'une face seule (kg)
lb = 0.05;         % Bras de levier (m) - Distance Sommet -> CG
I_face = 0.003;    % Inertie de la plaque 2D
Cb = 0.0008;       % Frottement de l'air

% --- Configuration Simulation ---
dt = 0.005; 
t_final = 3; 
n_steps = t_final/dt;

theta = zeros(1, n_steps); 
theta_dot = zeros(1, n_steps); 
time = zeros(1, n_steps);

% Condition initiale
theta(1) = deg2rad(5); % Perturbation de 5°

% --- Contrôle PID --- 
Kp = 7.0;  
Kd = 1.8; 
Ki = 0.3; 
integral_error = 0;
sat_limit = 0.08; 

% --- Simulation ---
for k = 1:n_steps-1
    error = 0 - theta(k);
    integral_error = integral_error + error * dt;
    
    % Calcul du couple moteur
    Tm_pid = Kp*error + Ki*integral_error - Kd*theta_dot(k);
    Tm = max(min(Tm_pid, sat_limit), -sat_limit); 
    
    % Dynamique du pendule inversé 2D
    % On utilise sin(theta) pour la non-linéarité du poids
    theta_ddot = (m_face*g*lb * sin(theta(k)) + Tm - Cb*theta_dot(k)) / I_face;
    
    % Intégration Euler
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1) = theta(k) + theta_dot(k) * dt;
    time(k+1) = time(k) + dt;
end

% --- Affichage Graphique Unique ---
figure('Color', 'w', 'Position', [100, 100, 900, 500]); % Fenêtre large
plot(time, rad2deg(theta), 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5);

grid on; hold on;
line([0 t_final], [0 0], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.2);

% Esthétique du graphe
title('Stabilisation d''une Face 2D (Perturbation initiale : 5°)', 'FontSize', 14);
xlabel('Temps (secondes)', 'FontSize', 12);
ylabel('Angle d''inclinaison (degrés)', 'FontSize', 12);
set(gca, 'FontSize', 11);
ylim([-1 6]); % Zoom sur la zone utile
xlim([0 t_final]);