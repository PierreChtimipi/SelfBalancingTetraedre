%% Expérience 2 : Redressement Réaliste (70.5° -> 0°)
clear all; close all; clc;

% --- Paramètres Physiques Réels ---
g = 9.81; 
mb = 0.5;       % 500g
lb = 0.075;      % Distance CG
Ieq = 0.006;     % Inertie réaliste (évite les vitesses infinies)
Cb = 0.001;      % Amortissement mécanique

% --- Configuration ---
dt = 0.005; t_final = 4; n_steps = t_final/dt;
theta = zeros(1, n_steps); theta_dot = zeros(1, n_steps); time = zeros(1, n_steps);
theta(1) = deg2rad(70.5); % Angle de départ (posé sur une face)

% --- Commande ---
% On utilise un couple POSITIF pour remonter vers 0
Tm_kick = 0.10;        % Couple max réel d'un EMAX RS2205
duration_kick = 0.500;  % On pousse pendant 500ms pour vaincre la gravité

for k = 1:n_steps-1
    if time(k) < duration_kick
        Tm = Tm_kick; 
    else
        % PID de stabilisation une fois proche de la verticale
        Kp = 7; Kd = 1.8;
        error = 0 - theta(k);
        Tm_pid = Kp*error - Kd*theta_dot(k);
        Tm = max(min(Tm_pid, 0.10), -0.10); 
    end
    
    % Équation : Accélération = (Couple Gravité + Couple Moteur - Friction) / Inertie
    theta_ddot = (-mb*g*lb * sin(theta(k)) + Tm - Cb*theta_dot(k)) / Ieq;
    
    % Intégration
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1) = theta(k) + theta_dot(k) * dt;
    time(k+1) = time(k) + dt;
    
    % Sécurité pour le graphique
    if rad2deg(theta(k+1)) < -30 || rad2deg(theta(k+1)) > 100; break; end
end

% --- Affichage ---
figure('Color', 'w');
plot(time(1:k), rad2deg(theta(1:k)), 'm', 'LineWidth', 2);
grid on; hold on;
line([0 t_final], [0 0], 'Color', 'k', 'LineStyle', '--');
title('Exp 2 : Redressement Réaliste (Base 70.5° vers Pointe 0°)');
xlabel('Temps (s)'); ylabel('Angle (°)');
ylim([-20 80]);