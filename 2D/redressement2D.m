%% Expérience 3 : Stabilisation Full 2D (Une seule face)
clear all; close all; clc;

% --- Paramètres Physiques FULL 2D (Une face seule) ---
g = 9.81; 
mb = 0.15;       % 150g : Masse d'une face seule + moteur/roue
lb = 0.050;      % 5.0cm : CG d'une face (plus bas qu'en 3D)
Ieq = 0.002;     % Inertie réduite pour une plaque 2D
Cb = 0.001;      

% --- Configuration ---
dt = 0.005; t_final = 3; n_steps = t_final/dt;
theta = zeros(1, n_steps); theta_dot = zeros(1, n_steps); time = zeros(1, n_steps);
theta(1) = deg2rad(70.5); 

% --- Commande ---
Tm_max = 0.12;   % Ton moteur EMAX réel

for k = 1:n_steps-1
    % PID avec paramètres adaptés à la nervosité de la 2D
    Kp = 7; 
    Kd = 1.8;
    error = 0 - theta(k);
    
    % Calcul du couple
    Tm_pid = Kp*error - Kd*theta_dot(k);
    Tm = max(min(Tm_pid, Tm_max), -Tm_max); 
    
    % Dynamique 2D
    % Note : Tm est positif pour remonter vers 0
    % mb*g*lb*sin(theta) est le couple de chute
    theta_ddot = (-mb*g*lb * sin(theta(k)) + Tm - Cb*theta_dot(k)) / Ieq;
    
    % Intégration
    theta_dot(k+1) = theta_dot(k) + theta_ddot * dt;
    theta(k+1) = theta(k) + theta_dot(k) * dt;
    time(k+1) = time(k) + dt;
    
    % Sécurité
    if rad2deg(theta(k+1)) < -20 || rad2deg(theta(k+1)) > 100; break; end
end

% --- Affichage ---
figure('Color', 'w', 'Position', [100 100 900 500]);
plot(time(1:k), rad2deg(theta(1:k)), 'b', 'LineWidth', 2.5);
grid on; hold on;
line([0 t_final], [0 0], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);

title('Stabilisation Full 2D : Une seule face (Moteur 0.12Nm)');
xlabel('Temps (s)'); ylabel('Angle (°)');
ylim([-5 80]); xlim([0 2]);

% Vérification mathématique du succès
C_gravite_max = mb * g * lb * sin(deg2rad(70.5));
fprintf('Couple Gravité Max : %.3f Nm\n', C_gravite_max);
fprintf('Couple Moteur Max : %.3f Nm\n', Tm_max);
if Tm_max > C_gravite_max
    disp('STATUT : Redressement physiquement possible !');
else
    disp('STATUT : Toujours trop lourd.');
end