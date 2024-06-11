% Importer les bibliothèques nécessaires
clear;
clc;
close all;

% Paramètres de la vidéo
fps = 10; % Frames par seconde
duration = 25; % Durée de la vidéo en secondes

% Créer une nouvelle vidéo
video = VideoWriter('trajectory_video_1_evtol3obstacle1target.mp4', 'MPEG-4');
video.FrameRate = fps;
open(video);

% Initialiser la figure
fig = figure;
hold on;

% Définir la largeur et la hauteur de la figure
fig_width = 800; % Largeur de la figure en pixels
fig_height = 500; % Hauteur de la figure en pixels
set(fig, 'Position', [400, 150, fig_width, fig_height]);

% Dessiner l'environnement 2D sans les rectangles et cercles pour l'eVTOL
Environment_1evtol1target3obstacle;

% Initialiser les variables pour les handles de l'eVTOL et des cercles
    
 global ev_handle;
 ev_handle = [];
 global circle_handles;
 circle_handles = [];
 
 global text_handles;
 text_handles = [];
 global ev_sensor_range;
 ev_sensor_range = [];
 
 global rayon_ev_carre;
 rayon_ev_carre = 0.04; % Rayon du carré
 
 global rayon_ev_cercle;
 rayon_ev_cercle = 0.02; % Rayon des cercles
 
 global rayon_evtol;
 rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
 
 global Seuil;
 Seuil = 0.02;
 
 global cpt;
 cpt=0;

 global NewEvtolParam;
 NewEvtolParam;
 
 global IdMin1;
 global previus_IdMin;
 
 global path;
 path=0.02;

% Dessiner les obstacles(cercle noir)
%Dessiner le rond noir pour Obstacle dans la légende
[X_obstacle_1,Y_obstacle_1] = ObstaclePosition(0.8,2);
[X_obstacle_2,Y_obstacle_2] = ObstaclePosition(0.8,2);
[X_obstacle_3,Y_obstacle_3] = ObstaclePosition(0.8,2);

plot(X_obstacle_1,Y_obstacle_1,'ko', 'MarkerSize', 4, 'LineWidth',8, 'DisplayName', 'Obstacle1');
plot(X_obstacle_2,Y_obstacle_2,'ro', 'MarkerSize', 4, 'LineWidth',8, 'DisplayName', 'Obstacle2');
plot(X_obstacle_3,Y_obstacle_3,'go', 'MarkerSize', 4, 'LineWidth',8, 'DisplayName', 'Obstacle3');
   
x_temp = x_evtol; 
y_temp = y_evtol;
previus_x_temp=x_evtol;
% Dessiner la trajectoire vers la première cible
tic;
while euclideanDistance(x_temp,y_temp,x_target1,y_target1)> Seuil
    
   
    
    
    % Générer progressivement les positions de l'eVTOL vers la première cible
    x_ev_trajectory1 = linspace(x_temp, x_target1, 50);
    y_ev_trajectory1 = linspace(y_temp, y_target1, 50);
    
    x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
    y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
    [Min1,IdMin1] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
    [Min2,IdMin2] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_2,Y_obstacle_2);
    [Min3,IdMin3] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);

    
    
    while((Min1 <= rayon_evtol)&&(euclideanDistance(x_temp,y_temp,x_target1,y_target1)>euclideanDistance(x_temp,y_temp,X_obstacle_1,Y_obstacle_1)))
          
    if (IdMin1==previus_IdMin)
        cpt=cpt+1;
    end
    
    if (cpt ==50)
        
        
       [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
       while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
             x_ev_trajectory1 = linspace(x_temp, X_emergency_Waypoint, 50);
             y_ev_trajectory1 = linspace(y_temp, Y_emergency_Waypoint, 50);
    
             x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
             y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
            text_handles=text(0.3, 1, 'Emergency protocol reaching security altitude', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
        end
        cpt=0; 
        
    end
    
   
    display(cpt);
    switch IdMin1
        
        case 1
           x_temp = x_temp +path; 
           y_temp = y_temp +path;
           
           previus_IdMin=1;
           SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
       
            % Dessiner le nouveau carré de l'eVTOL
             rayon_ev_carre = 0.04; % Rayon du carré
             rayon_ev_cercle = 0.02; % Rayon des cercles
             x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
             y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
             ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
           break
           
        case 2
            x_temp = x_temp + path; 
            y_temp = y_temp -path;
            previus_IdMin=2;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
                    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end


            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        case 3
            x_temp = x_temp - path; 
            y_temp = y_temp + path;
            previus_IdMin=3;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
            
          
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        otherwise
             x_temp = x_temp - path; 
             y_temp = y_temp - path;
             previus_IdMin=4;
             SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
                
             % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
                                       
             % Capturer l'image actuelle pour la vidéo
             frame = getframe(fig);
             writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
             break;
    end
    
    end
    while((Min2 <= rayon_evtol)&&(euclideanDistance(x_temp,y_temp,x_target1,y_target1)>euclideanDistance(x_temp,y_temp,X_obstacle_2,Y_obstacle_2)))
          
    if (IdMin2==previus_IdMin)
        cpt=cpt+1;
    end
    
    if (cpt ==50)
        
        
       [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
       while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
             x_ev_trajectory1 = linspace(x_temp, X_emergency_Waypoint, 50);
             y_ev_trajectory1 = linspace(y_temp, Y_emergency_Waypoint, 50);
    
             x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
             y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
            text_handles=text(0.3, 1, 'Emergency protocol reaching security altitude', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
        end
        cpt=0; 
        
    end
    
    switch IdMin2
        
        case 1
           x_temp = x_temp +0.01; 
           y_temp = y_temp + 0.01;
           
           previus_IdMin=1;
           SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_2,Y_obstacle_2);
       
            % Dessiner le nouveau carré de l'eVTOL
             rayon_ev_carre = 0.04; % Rayon du carré
             rayon_ev_cercle = 0.02; % Rayon des cercles
             x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
             y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
             ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
           break
           
        case 2
            x_temp = x_temp + 0.01; 
            y_temp = y_temp -0.01;
            previus_IdMin=2;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_2,Y_obstacle_2);
                    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end


            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        case 3
            x_temp = x_temp - 0.01; 
            y_temp = y_temp + 0.01;
            previus_IdMin=3;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_2,Y_obstacle_2);
            
          
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        otherwise
             x_temp = x_temp - 0.01; 
             y_temp = y_temp - 0.01;
             previus_IdMin=4;
             SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_2,Y_obstacle_2);
                
             % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
                                       
             % Capturer l'image actuelle pour la vidéo
             frame = getframe(fig);
             writeVideo(video, frame);
             deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
             break;
    end
    
    end
    while((Min3 <= rayon_evtol)&&(euclideanDistance(x_temp,y_temp,x_target1,y_target1)>euclideanDistance(x_temp,y_temp,X_obstacle_3,Y_obstacle_3)))
          
    if (IdMin3==previus_IdMin)
        cpt=cpt+1;
    end
    
    if (cpt ==50)
        
        
       [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
       while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
             x_ev_trajectory1 = linspace(x_temp, X_emergency_Waypoint, 50);
             y_ev_trajectory1 = linspace(y_temp, Y_emergency_Waypoint, 50);
    
             x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
             y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
            text_handles=text(0.3, 1, 'Emergency protocol reaching security altitude', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
        end
        cpt=0; 
        
    end
    
    switch IdMin3
        
        case 1
           x_temp = x_temp +path; 
           y_temp = y_temp + path;
           
           previus_IdMin=1;
           SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
       
            % Dessiner le nouveau carré de l'eVTOL
             rayon_ev_carre = 0.04; % Rayon du carré
             rayon_ev_cercle = 0.02; % Rayon des cercles
             x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
             y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
             ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
           break
           
        case 2
            x_temp = x_temp + path; 
            y_temp = y_temp -path;
            previus_IdMin=2;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
                    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end


            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        case 3
            x_temp = x_temp - path; 
            y_temp = y_temp + path;
            previus_IdMin=3;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
            
          
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        otherwise
             x_temp = x_temp - path; 
             y_temp = y_temp - path;
             previus_IdMin=4;
             SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
                
             % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
                                       
             % Capturer l'image actuelle pour la vidéo
             frame = getframe(fig);
             writeVideo(video, frame);
             deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
             break;
    end
    end

    
    % Dessiner le nouveau carré de l'eVTOL
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
    y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
    ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
    rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
    ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
    % Dessiner les cercles aux coins du nouveau carré
    circle_handles = []; % Garder une trace des handles des cercles
    for j = 1:4
        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
        circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    end

   % Capturer l'image actuelle pour la vidéo
   frame = getframe(fig);
   writeVideo(video, frame);
   % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
   deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
end  

%initialisation départ trajectoire
x_temp = x_target1; 
y_temp = y_target1;

x_vertiPort= 0.1+rayon_evtol;
y_vertiPort= 1;

% Dessiner la trajectoire retour
while euclideanDistance(x_temp,y_temp,X_Emergency,Y_Emergency)> Seuil
    
    % Générer progressivement les positions de l'eVTOL vers la première cible
    x_ev_trajectory2 = linspace(x_temp, X_Emergency, 50);
    y_ev_trajectory2 = linspace(y_temp, Y_Emergency, 50);
    
    x_temp = x_temp + x_ev_trajectory2(2)-x_temp; 
    y_temp = y_temp + y_ev_trajectory2(2)-y_temp;
    
   [Min1,IdMin1] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
   [Min2,IdMin2] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_2,Y_obstacle_2);
   [Min3,IdMin3] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
     
    while ((Min1 <= rayon_evtol)&&(euclideanDistance(x_temp,y_temp,X_Emergency,Y_Emergency)>euclideanDistance(x_temp,y_temp,X_obstacle_1,Y_obstacle_1)))
          
    if (IdMin1==previus_IdMin)
        cpt=cpt+1;
        display(cpt);
    end
    
    if (cpt ==50)
       
       [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
       while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
            x_ev_trajectory1 = linspace(x_temp, X_emergency_Waypoint, 50);
            y_ev_trajectory1 = linspace(y_temp, Y_emergency_Waypoint, 50);
    
            x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
            y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
            text_handles=text(0.3, 1, 'Emergency protocol reaching security altitude', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
        end
        cpt=0; 
        
    end
    switch IdMin1
        
        case 1
            
           x_temp = x_temp +path; 
           y_temp = y_temp + path;
           previus_IdMin=1;  


            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
              % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break
        case 2
               
            x_temp = x_temp + path; 
            y_temp = y_temp - path;
            previus_IdMin=2;
            
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
              % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            
            break;
            
        case 3
            
            x_temp = x_temp - path; 
            y_temp = y_temp + path;
            previus_IdMin=3;
            
            sensorBottomLeftO1 = euclideanDistance(x_temp - rayon_ev_carre,y_temp - rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
                            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
              % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        otherwise
           
             x_temp = x_temp - path; 
             y_temp = y_temp - path;
             previus_IdMin=4;
             
             SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
             % Dessiner le nouveau carré de l'eVTOL
             rayon_ev_carre = 0.04; % Rayon du carré
             rayon_ev_cercle = 0.02; % Rayon des cercles
             x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
             y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
             ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

             % Générer le cercle autour de l'eVTOL (sensor range)
             centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
             rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
             ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

             % Dessiner les cercles aux coins du nouveau carré
             circle_handles = []; % Garder une trace des handles des cercles
             for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
             end
             % Capturer l'image actuelle pour la vidéo
             frame = getframe(fig);
             writeVideo(video, frame);
             deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles); 
             break;
    end
    
    end
    while ((Min2 <= rayon_evtol)&&(euclideanDistance(x_temp,y_temp,X_Emergency,Y_Emergency)>euclideanDistance(x_temp,y_temp,X_obstacle_2,Y_obstacle_2)))
          
    if (IdMin2==previus_IdMin)
        cpt=cpt+1;
        display(cpt);
    end
    
    if (cpt ==50)

       
        
       [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
       while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
            x_ev_trajectory1 = linspace(x_temp, X_emergency_Waypoint, 50);
            y_ev_trajectory1 = linspace(y_temp, Y_emergency_Waypoint, 50);
    
            x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
            y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
            text_handles=text(0.3, 1, 'Emergency protocol reaching security altitude', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
        end
        cpt=0; 
        
    end
    switch IdMin2
        
        case 1
            
           x_temp = x_temp +path; 
           y_temp = y_temp + path;
           previus_IdMin=1;  


            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
              % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break
        case 2
               
            x_temp = x_temp + path; 
            y_temp = y_temp - path;
            previus_IdMin=2;
            
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
              % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            
            break;
            
        case 3
            
            x_temp = x_temp - path; 
            y_temp = y_temp +path;
            previus_IdMin=3;
            
            sensorBottomLeftO1 = euclideanDistance(x_temp - rayon_ev_carre,y_temp - rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
                            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
              % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        otherwise
           
             x_temp = x_temp - path; 
             y_temp = y_temp - path;
             previus_IdMin=4;
             
             SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_1,Y_obstacle_1);
             % Dessiner le nouveau carré de l'eVTOL
             rayon_ev_carre = 0.04; % Rayon du carré
             rayon_ev_cercle = 0.02; % Rayon des cercles
             x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
             y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
             ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

             % Générer le cercle autour de l'eVTOL (sensor range)
             centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
             rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
             ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);

             % Dessiner les cercles aux coins du nouveau carré
             circle_handles = []; % Garder une trace des handles des cercles
             for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
             end
             % Capturer l'image actuelle pour la vidéo
             frame = getframe(fig);
             writeVideo(video, frame);
             deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles); 
             break;
    end
    
    end
    while((Min3 <= rayon_evtol)&&(euclideanDistance(x_temp,y_temp,X_Emergency,Y_Emergency)>euclideanDistance(x_temp,y_temp,X_obstacle_3,Y_obstacle_3)))
          
    if (IdMin3==previus_IdMin)
        cpt=cpt+1;
    end
    
    if (cpt ==50)
        
        
       [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
       while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
             x_ev_trajectory1 = linspace(x_temp, X_emergency_Waypoint, 50);
             y_ev_trajectory1 = linspace(y_temp, Y_emergency_Waypoint, 50);
    
             x_temp = x_temp + (x_ev_trajectory1(2)-x_temp); 
             y_temp = y_temp + (y_ev_trajectory1(2)-y_temp);
    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
            text_handles=text(0.3, 1, 'Emergency protocol reaching security altitude', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');

            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
            
            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
        end
        cpt=0; 
        
    end
    
    switch IdMin3
        
        case 1
           x_temp = x_temp +path; 
           y_temp = y_temp + path;
           
           previus_IdMin=1;
           SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
       
            % Dessiner le nouveau carré de l'eVTOL
             rayon_ev_carre = 0.04; % Rayon du carré
             rayon_ev_cercle = 0.02; % Rayon des cercles
             x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
             y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
             ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
           break
           
        case 2
            x_temp = x_temp + path; 
            y_temp = y_temp - path;
            previus_IdMin=2;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
                    
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end


            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        case 3
            x_temp = x_temp - path; 
            y_temp = y_temp + path;
            previus_IdMin=3;
            SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
            
          
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Capturer l'image actuelle pour la vidéo
            frame = getframe(fig);
            writeVideo(video, frame);
            deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
            break;
            
        otherwise
             x_temp = x_temp - path; 
             y_temp = y_temp - path;
             previus_IdMin=4;
             SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_obstacle_3,Y_obstacle_3);
                
             % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
            y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
            ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    
            % Dessiner les cercles aux coins du nouveau carré
            circle_handles = []; % Garder une trace des handles des cercles
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
                                       
             % Capturer l'image actuelle pour la vidéo
             frame = getframe(fig);
             writeVideo(video, frame);
             deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
             break;
    end
    end
    % Dessiner le nouveau carré de l'eVTOL
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
    y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
    ev_handle = rectangle('Position', [x_temp - rayon_ev_carre, y_temp - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
    rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
    ev_sensor_range=viscircles(centre_evtol, rayon_evtol, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
    % Dessiner les cercles aux coins du nouveau carré
    circle_handles = []; % Garder une trace des handles des cercles
    for j = 1:4
        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
        circle_handles(j) = plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    end

    % Capturer l'image actuelle pour la vidéo
    frame = getframe(fig);
    writeVideo(video, frame);
    % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
    deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles);
end
toc;
% Fermer la vidéo
close(video);
