%% Expérience 4 : Tentative de stabilisation avec élan (Swing-up)
clear all; close all; clc;

% --- Paramètres 3D RÉELS (STRICTS) ---
g = 9.81; mb = 0.5; lb = 0.075; Ieq = 0.006; Cb = 0.005;
dt = 0.005; t_final = 2; n_steps = t_final/dt;
theta = zeros(1, n_steps); theta_dot = zeros(1, n_steps); time = zeros(1, n_steps);

% --- ÉLAN INITIAL ---
theta(1) = deg2rad(70.5); 
% --- Paramètres de la Roue de Réaction ---
m_roue = 0.080;        % 80g
r_roue = 0.030;        % Rayon 3cm
I_roue = 0.5 * m_roue * r_roue^2; % Inertie disque : 1/2*m*r^2
Vitesse_RPM = 8000;    % Vitesse de la roue avant le freinage
Omega_roue = Vitesse_RPM * 2 * pi / 60;

% L'impulsion transforme le moment de la roue en vitesse pour le bloc
theta_dot(1) = -(I_roue * Omega_roue) / Ieq; 

fprintf('Vitesse de remontée générée par le choc : %.2f rad/s\n', theta_dot(1));
% --- Commande Limite ---
Tm_max = 0.12; 
Kp = 7; Kd = 1.8; % On booste le Kd pour freiner

for k = 1:n_steps-1
    error = 0 - theta(k);
    Tm_pid = Kp*error - Kd*theta_dot(k);
    Tm = max(min(Tm_pid, Tm_max), -Tm_max); 
    
    % Dynamique
    theta_ddot = (-mb*g*lb * sin(theta(k)) + Tm - Cb*theta_dot(k)) / Ieq;
    
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1) = theta(k) + theta_dot(k) * dt;
    time(k+1) = time(k) + dt;
    
    if abs(rad2deg(theta(k+1))) > 100, break; end
end

figure('Color', 'w', 'Position', [100 100 800 450]);
plot(time(1:k), rad2deg(theta(1:k)), 'b', 'LineWidth', 2);
grid on; hold on;
line([0 t_final], [0 0], 'Color', 'k', 'LineStyle', '--');
title('Stabilisation 3D avec élan (Swing-up)');
xlabel('Temps (s)'); ylabel('Angle (°)');