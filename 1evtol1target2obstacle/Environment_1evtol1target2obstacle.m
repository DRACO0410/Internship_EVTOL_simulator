% Initialiser la figure
fig = figure;
hold on;
% Définir la largeur et la hauteur de la figure
fig_width = 800; % Largeur de la figure en pixels
fig_height = 600; % Hauteur de la figure en pixels
set(fig, 'Position', [400, 150, fig_width, fig_height]);

% Longueur du côté du carré
side_length = 0.99;

% Coordonnées du coin en bas à gauche
x_bottom_left = 0;
y_bottom_left = 0;

% Calcul des coordonnées des coins du carré
x_corners = [x_bottom_left, x_bottom_left + side_length, x_bottom_left + side_length, x_bottom_left, x_bottom_left];
y_corners = [y_bottom_left, y_bottom_left, y_bottom_left + 0.8, y_bottom_left + 0.8, y_bottom_left];

% Tracé du carré
plot(x_corners, y_corners, 'g', 'LineWidth', 2);
x_danger = [1 2 2 1]; % Coordonnées en x du danger zone (km)
y_danger = [0 0 2 2]; % Coordonnées en y du danger zone (km)

% Tracé du Danger zone
plot(x_danger, y_danger, 'r', 'LineWidth', 2);

% Génération aléatoire des coordonnées de l'eVTOL
x_evtol = 0.4; % Génère un nombre aléatoire entre 0.25 et 0.75
y_evtol = 0.2 + rand * 0.2; % Génère un nombre aléatoire entre 0 et 2

% Paramètres du carré et des cercles
rayon_ev_carre = 0.04; % Rayon du carré
rayon_ev_cercle = 0.02; % Rayon des cercles
centre_range_ev_cercle= [0.4,y_evtol - rayon_ev_carre];
rayon_range_ev_cercle = 0.05;
% Dessiner le carré
%rectangle('Position', [x_evtol - rayon_ev_carre, y_evtol - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

% Générer le cercle autour de l'eVTOL (sensor range)
centre_evtol = [0.4, y_evtol]; % Coordonnées du centre de l'eVTOL
rayon_evtol = 0.1; % Rayon du cercle autour de l'eVTOL
%viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

% Calculer les coordonnées des coins du carré
x_corners = [x_evtol - rayon_ev_carre, x_evtol + rayon_ev_carre, x_evtol + rayon_ev_carre, x_evtol - rayon_ev_carre];
y_corners = [y_evtol - rayon_ev_carre, y_evtol - rayon_ev_carre, y_evtol + rayon_ev_carre, y_evtol + rayon_ev_carre];

% Dessiner les cercles aux coins
for i = 1:4
    % Tracer le cercle
    
    theta_circle = linspace(0, 2*pi, 100);
    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
    %plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
end

% Génération aléatoire des coordonnées de la cible en danger
x_target1 = 1.8; % Génère un nombre aléatoire entre 1.25 et 1.75
y_target1 = 0.8; % Génère un nombre aléatoire entre 0 et 2

% Tracé de la croix représentant la cible
plot([x_target1-0.05 x_target1+0.05], [y_target1 y_target1], 'k', 'LineWidth', 2); % Tracé de la ligne horizontale
plot([x_target1 x_target1], [y_target1-0.05 y_target1+0.05], 'k', 'LineWidth', 2); % Tracé de la ligne verticale

% Dessiner l'obstacle (cercle noir)
%plot(x_obstacle, y_obstacle, 'ok', 'MarkerSize', 12, 'LineWidth', 12);

% Tracé de la droite verticale orange
x_vertical_line = [0.4 0.4]; % Coordonnées en x
y_vertical_line = [0.2 0.4]; % Coordonnées en y

% Ajout de titres et légendes
title('2D environment');
xlabel('X position (km)');
ylabel('Y position (km)');
legend('Vertiport','Danger zone','Location','northeastoutside');

% Tracé du trait vertical avec un style de ligne en pointillés et une couleur grise
plot([1 1], [0 2], 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Imaginary Boundary');
plot(x_vertical_line, y_vertical_line, 'Color', [1, 0.5, 0], 'LineWidth', 2, 'DisplayName', 'Take off zone');

% Dessiner le rond bleu pour Emergency dans la légende
X_Emergency=0.1+rayon_evtol;
Y_Emergency=1+rayon_evtol;
plot(X_Emergency, Y_Emergency, 'bo', 'MarkerSize', 4, 'LineWidth',8, 'DisplayName', 'Emergency');

%Dessiner le rond noir pour Obstacle dans la légende
%plot(x_obstacle, y_obstacle,'ko', 'MarkerSize', 4, 'LineWidth',8, 'DisplayName', 'Obstacle');

grid on;







