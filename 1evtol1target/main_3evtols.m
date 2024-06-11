% Importer les bibliothèques nécessaires
clear;
clc;
close all;

% Paramètres de la vidéo
fps = 10; % Frames par seconde
duration = 25; % Durée de la vidéo en secondes

% Créer une nouvelle vidéo
video = VideoWriter('trajectory_video_2evtol.mp4', 'MPEG-4');
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
Environment_2evtol;

% Initialiser les variables pour les handles de l'eVTOL et des cercles
 global  ValSensorRange; 
 ValSensorRange=0.2;

 global ev_handle_1;
 ev_handle_1 = [];
 
 global circle_handles_1;
 circle_handles_1 = [];
 
 global text_handles_1;
 text_handles_1 = [];
 
 global ev_sensor_range_1;
 ev_sensor_range_1 = [];
 
 global ev_handle_2;
 ev_handle_2 = [];
 
 global circle_handles_2;
 circle_handles_2 = [];
 
 global text_handles_2;
 text_handles_2 = [];
 
 global ev_sensor_range_2;
 ev_sensor_range_2 = [];
 
 global ev_handle_3;
 ev_handle_3 = [];
 
 global circle_handles_3;
 circle_handles_3 = [];
 
 global text_handles_3;
 text_handles_3 = [];
 
 global ev_sensor_range_3;
 ev_sensor_range_3 = [];
 
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
 
 global Flag_arrived_target_1;
 Flag_arrived_target_1 = 0;
 
 global Flag_arrived_target_2;
 Flag_arrived_target_2 = 0;
 
 global Flag_arrived_target_3;
 Flag_arrived_target_3 = 0;
 
 global Flag_arrived_base_1;
 Flag_arrived_base_1 = 0;
 
 global Flag_arrived_base_2;
 Flag_arrived_base_2 = 0;
 
  global Flag_arrived_base_3;
 Flag_arrived_base_3 = 0;
 
 global IdMin1;
 global IdMin2;
 global IdMin3;
 global previus_IdMin;
 global previus_IdMin_2;
 global previus_IdMin_3;
 %init position evtol
 x_temp_1 = x_evtol_1; 
 y_temp_1 = y_evtol_1;

 x_temp_2 = x_evtol_2; 
 y_temp_2 = y_evtol_2;
 
 x_temp_3 = x_evtol_3;
 y_temp_3 = y_evtol_3;
 
 while(( Flag_arrived_target_1 == 0)&&( Flag_arrived_target_2 == 0))
    
    if (euclideanDistance(x_temp_1,y_temp_1,x_target_1,y_target_1)> Seuil) 
       
        x_ev_trajectory1 = linspace(x_temp_1, x_target_1, 50);
        y_ev_trajectory1 = linspace(y_temp_1, y_target_1, 50);
    
        x_temp_1 = x_temp_1 + (x_ev_trajectory1(2)-x_temp_1); 
        y_temp_1 = y_temp_1 + (y_ev_trajectory1(2)-y_temp_1); 
     
    else
        Flag_arrived_target_1 = 1;        
    end 
    
    if (euclideanDistance(x_temp_2,y_temp_2,x_target_2,y_target_2)> Seuil)
       
        x_ev_trajectory2 = linspace(x_temp_2, x_target_2, 50);
        y_ev_trajectory2 = linspace(y_temp_2, y_target_2, 50);
    
        x_temp_2 = x_temp_2 + (x_ev_trajectory2(2)-x_temp_2); 
        y_temp_2 = y_temp_2 + (y_ev_trajectory2(2)-y_temp_2);
    
    else
         Flag_arrived_target_2 = 1;   
    end
    
    if (euclideanDistance(x_temp_3,y_temp_3,x_target_3,y_target_3)> Seuil)
       
        x_ev_trajectory3 = linspace(x_temp_3, x_target_3, 50);
        y_ev_trajectory3 = linspace(y_temp_3, y_target_3, 50);
    
        x_temp_3 = x_temp_3 + (x_ev_trajectory3(2)-x_temp_3); 
        y_temp_3 = y_temp_3 + (y_ev_trajectory3(2)-y_temp_3);
    
    else
         Flag_arrived_target_3 = 1;   
    end
    
    if(euclideanDistance(x_temp_2,y_temp_2,x_temp_1,y_temp_1)< 2*rayon_evtol)
        
       while(euclideanDistance(x_temp_2,y_temp_2,x_temp_1,y_temp_1)<2*rayon_evtol)

        
           if (euclideanDistance(x_temp_1,y_temp_1,x_target_1,y_target_1)>(euclideanDistance(x_temp_2,y_temp_2,x_target_2,y_target_2)))

                   [Min1,IdMin1] = SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);
                   display(IdMin1);
                   while((Min1 <= 2*rayon_evtol)&&(x_temp_2<x_target_1)&((Min1 <= 2*rayon_evtol)&&(y_temp_2<y_target_1)))

                        if (IdMin1==previus_IdMin)
                            cpt=cpt+1;
                        end
    
                       if (cpt ==80)
        
        
                        [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
                        while euclideanDistance(x_temp_1,y_temp_1,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
                                  % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                        y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                        ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré
                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                        y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                        ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré

                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end
                        % Paramètres du carré et des cercles
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                        rayon_range_ev_cercle = 0.05;
                        % Dessiner le carré
                        rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                        rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                        viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Calculer les coordonnées des coins du carré
                        x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                        y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                        % Dessiner les cercles aux coins
                        for i = 1:4
                            % Tracer le cercle

                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                            plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                        end


                       % Capturer l'image actuelle pour la vidéo
                       frame = getframe(fig);
                       writeVideo(video, frame);
                       % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                       deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                       deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
                   
                       end
                        cpt=0; 
        
                       end
    
   
                        display(cpt);
                        switch IdMin1
        
                        case 1
                           x_temp_1 = x_temp_1 +0.01; 
                           y_temp_1 = y_temp_1 + 0.01;

                           previus_IdMin=1;
                           SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
                           break

                        case 2
                            x_temp_1 = x_temp_1 + 0.01; 
                            y_temp_1 = y_temp_1 -0.01;
                            previus_IdMin=2;
                            SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);

                            % Dessiner le nouveau carré de l'eVTOL

                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré
                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Dessiner le nouveau carré de l'eVTOL
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré

                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Paramètres du carré et des cercles
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                            rayon_range_ev_cercle = 0.05;
                            % Dessiner le carré
                            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Calculer les coordonnées des coins du carré
                            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                            % Dessiner les cercles aux coins
                            for i = 1:4
                                % Tracer le cercle

                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                            end

                           % Capturer l'image actuelle pour la vidéo
                           frame = getframe(fig);
                           writeVideo(video, frame);
                           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                           break;

                        case 3
                            x_temp_1 = x_temp_1 - 0.01; 
                            y_temp_1 = y_temp_1 + 0.01;
                            previus_IdMin=3;
                            SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);


                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                            break;

                        otherwise
                             x_temp_1 = x_temp_1 - 0.01; 
                             y_temp_1 = y_temp_1 - 0.01;
                             previus_IdMin=4;
                             SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);

                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                             break;
                        end                        
                    end
           else
  
                   [Min2,IdMin2] = SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);
                   while((Min2 <= 2*rayon_evtol)&&(x_temp_1<x_target_2)&((Min2 <= 2*rayon_evtol)&&(y_temp_1<y_target_2)))

                        if (IdMin2==previus_IdMin_2)
                            cpt=cpt+1;
                        end
    
                        if (cpt ==80)
        
        
                        [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
                        while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                        y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                        ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré
                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                        y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                        ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré

                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Paramètres du carré et des cercles
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                        rayon_range_ev_cercle = 0.05;
                        % Dessiner le carré
                        rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                        rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                        viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Calculer les coordonnées des coins du carré
                        x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                        y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                        % Dessiner les cercles aux coins
                        for i = 1:4
                            % Tracer le cercle

                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                            plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                        end


                       % Capturer l'image actuelle pour la vidéo
                       frame = getframe(fig);
                       writeVideo(video, frame);
                       % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                       deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                       deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
                   
                        end
                        cpt=0; 
        
                        end
    
   
                        display(cpt);
                        switch IdMin2
        
                        case 1
                           x_temp_2 = x_temp_2 +0.01; 
                           y_temp_2 = y_temp_2 + 0.01;

                           previus_IdMin_2=1;
                           SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
                           break

                        case 2
                            x_temp_2 = x_temp_2 + 0.01; 
                            y_temp_2 = y_temp_2 -0.01;
                            previus_IdMin_2=2;
                            SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                            % Dessiner le nouveau carré de l'eVTOL

                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré
                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Dessiner le nouveau carré de l'eVTOL
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré

                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Paramètres du carré et des cercles
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                            rayon_range_ev_cercle = 0.05;
                            % Dessiner le carré
                            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Calculer les coordonnées des coins du carré
                            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                            % Dessiner les cercles aux coins
                            for i = 1:4
                                % Tracer le cercle

                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                            end


                           % Capturer l'image actuelle pour la vidéo
                           frame = getframe(fig);
                           writeVideo(video, frame);
                           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                           break;

                        case 3
                            x_temp_2 = x_temp_2 - 0.01; 
                            y_temp_2 = y_temp_2 + 0.01;
                            previus_IdMin_2=3;
                            SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);


                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                            break;

                        otherwise
                             x_temp_2 = x_temp_2 - 0.01; 
                             y_temp_2 = y_temp_2 - 0.01;
                             previus_IdMin_2=4;
                             SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                             break;
                        end                        
                    end
           end
     
                 % Dessiner le nouveau carré de l'eVTOL
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Dessiner les cercles aux coins du nouveau carré
                for j = 1:4
                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                end

                % Dessiner le nouveau carré de l'eVTOL
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Dessiner les cercles aux coins du nouveau carré

                for j = 1:4
                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                end

                % Paramètres du carré et des cercles
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                rayon_range_ev_cercle = 0.05;
                % Dessiner le carré
                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Calculer les coordonnées des coins du carré
                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                % Dessiner les cercles aux coins
                for i = 1:4
                    % Tracer le cercle

                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                end


               % Capturer l'image actuelle pour la vidéo
               frame = getframe(fig);
               writeVideo(video, frame);
               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
        end  

    else
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré

            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Paramètres du carré et des cercles
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
            rayon_range_ev_cercle = 0.05;
            % Dessiner le carré
            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Calculer les coordonnées des coins du carré
            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

            % Dessiner les cercles aux coins
            for i = 1:4
                % Tracer le cercle

                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
            
               
    end
    
    if(euclideanDistance(x_temp_2,y_temp_2,x_temp_3,y_temp_3)< 2*rayon_evtol)
        
       while(euclideanDistance(x_temp_2,y_temp_2,x_temp_3,y_temp_3)<2*rayon_evtol)

        
           if (euclideanDistance(x_temp_3,y_temp_3,x_target_3,y_target_3)>(euclideanDistance(x_temp_2,y_temp_2,x_target_2,y_target_2)))

                   [Min3,IdMin3] = SensorDetectionForObstacle(x_temp_3,y_temp_3,rayon_ev_carre,x_temp_2,y_temp_2);
                   display(IdMin3);
                   while((Min3 <= 2*rayon_evtol)&&(x_temp_2<x_target_3)&((Min3 <= 2*rayon_evtol)&&(y_temp_2<y_target_3)))

                        if (IdMin3==previus_IdMin_3)
                            cpt=cpt+1;
                        end
    
                       if (cpt ==80)
        
        
                        [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
                        while euclideanDistance(x_temp_3,y_temp_3,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
                                  % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                        y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                        ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré
                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                        y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                        ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré

                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end
                        % Paramètres du carré et des cercles
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                        rayon_range_ev_cercle = 0.05;
                        % Dessiner le carré
                        rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                        rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                        viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Calculer les coordonnées des coins du carré
                        x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                        y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                        % Dessiner les cercles aux coins
                        for i = 1:4
                            % Tracer le cercle

                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                            plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                        end


                       % Capturer l'image actuelle pour la vidéo
                       frame = getframe(fig);
                       writeVideo(video, frame);
                       % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                       deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                       deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
                   
                       end
                        cpt=0; 
        
                       end
    
   
                        display(cpt);
                        switch IdMin1
        
                        case 1
                           x_temp_3 = x_temp_3 +0.01; 
                           y_temp_3 = y_temp_3 + 0.01;

                           previus_IdMin_3=1;
                           SensorDetectionForObstacle(x_temp_3,y_temp_3,rayon_ev_carre,x_temp_2,y_temp_2);

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
                           break

                        case 2
                            x_temp_3 = x_temp_3 + 0.01; 
                            y_temp_3 = y_temp_3 -0.01;
                            previus_IdMin_3=2;
                            SensorDetectionForObstacle(x_temp_3,y_temp_3,rayon_ev_carre,x_temp_2,y_temp_2);

                            % Dessiner le nouveau carré de l'eVTOL

                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré
                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Dessiner le nouveau carré de l'eVTOL
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré

                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Paramètres du carré et des cercles
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                            rayon_range_ev_cercle = 0.05;
                            % Dessiner le carré
                            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Calculer les coordonnées des coins du carré
                            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                            % Dessiner les cercles aux coins
                            for i = 1:4
                                % Tracer le cercle

                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                            end

                           % Capturer l'image actuelle pour la vidéo
                           frame = getframe(fig);
                           writeVideo(video, frame);
                           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                           break;

                        case 3
                            x_temp_3 = x_temp_3 - 0.01; 
                            y_temp_3 = y_temp_3 + 0.01;
                            previus_IdMin_3=3;
                            SensorDetectionForObstacle(x_temp_3,y_temp_3,rayon_ev_carre,x_temp_2,y_temp_2);


                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                            break;

                        otherwise
                             x_temp_3 = x_temp_3 - 0.01; 
                             y_temp_3 = y_temp_3 - 0.01;
                             previus_IdMin=4;
                             SensorDetectionForObstacle(x_temp_3,y_temp_3,rayon_ev_carre,x_temp_2,y_temp_2);

                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                             break;
                        end                        
                    end
           else
  
                   [Min2,IdMin2] = SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);
                   while((Min2 <= 2*rayon_evtol)&&(x_temp_1<x_target_2)&((Min2 <= 2*rayon_evtol)&&(y_temp_1<y_target_2)))

                        if (IdMin2==previus_IdMin_2)
                            cpt=cpt+1;
                        end
    
                        if (cpt ==80)
        
        
                        [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
                        while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                        y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                        ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré
                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                        y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                        ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré

                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Paramètres du carré et des cercles
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                        rayon_range_ev_cercle = 0.05;
                        % Dessiner le carré
                        rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                        rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                        viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Calculer les coordonnées des coins du carré
                        x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                        y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                        % Dessiner les cercles aux coins
                        for i = 1:4
                            % Tracer le cercle

                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                            plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                        end


                       % Capturer l'image actuelle pour la vidéo
                       frame = getframe(fig);
                       writeVideo(video, frame);
                       % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                       deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                       deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
                   
                        end
                        cpt=0; 
        
                        end
    
   
                        display(cpt);
                        switch IdMin2
        
                        case 1
                           x_temp_2 = x_temp_2 +0.01; 
                           y_temp_2 = y_temp_2 + 0.01;

                           previus_IdMin_2=1;
                           SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
                           break

                        case 2
                            x_temp_2 = x_temp_2 + 0.01; 
                            y_temp_2 = y_temp_2 -0.01;
                            previus_IdMin_2=2;
                            SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                            % Dessiner le nouveau carré de l'eVTOL

                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré
                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Dessiner le nouveau carré de l'eVTOL
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré

                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Paramètres du carré et des cercles
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                            rayon_range_ev_cercle = 0.05;
                            % Dessiner le carré
                            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Calculer les coordonnées des coins du carré
                            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                            % Dessiner les cercles aux coins
                            for i = 1:4
                                % Tracer le cercle

                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                            end


                           % Capturer l'image actuelle pour la vidéo
                           frame = getframe(fig);
                           writeVideo(video, frame);
                           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                           break;

                        case 3
                            x_temp_2 = x_temp_2 - 0.01; 
                            y_temp_2 = y_temp_2 + 0.01;
                            previus_IdMin_2=3;
                            SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);


                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                            break;

                        otherwise
                             x_temp_2 = x_temp_2 - 0.01; 
                             y_temp_2 = y_temp_2 - 0.01;
                             previus_IdMin_2=4;
                             SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                             break;
                        end                        
                    end
           end
     
                 % Dessiner le nouveau carré de l'eVTOL
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Dessiner les cercles aux coins du nouveau carré
                for j = 1:4
                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                end

                % Dessiner le nouveau carré de l'eVTOL
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Dessiner les cercles aux coins du nouveau carré

                for j = 1:4
                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                end

                % Paramètres du carré et des cercles
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                rayon_range_ev_cercle = 0.05;
                % Dessiner le carré
                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Calculer les coordonnées des coins du carré
                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                % Dessiner les cercles aux coins
                for i = 1:4
                    % Tracer le cercle

                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                end


               % Capturer l'image actuelle pour la vidéo
               frame = getframe(fig);
               writeVideo(video, frame);
               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
        end  

    else
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré

            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Paramètres du carré et des cercles
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
            rayon_range_ev_cercle = 0.05;
            % Dessiner le carré
            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Calculer les coordonnées des coins du carré
            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

            % Dessiner les cercles aux coins
            for i = 1:4
                % Tracer le cercle

                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
            
               
    end
    
     % Dessiner le nouveau carré de l'eVTOL
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
    y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
    ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
    
    % Dessiner les cercles aux coins du nouveau carré
    for j = 1:4
        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
        circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    
    % Dessiner le nouveau carré de l'eVTOL
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
    y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
    ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
    
    % Dessiner les cercles aux coins du nouveau carré

    for j = 1:4
        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
        circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
    end

    % Paramètres du carré et des cercles
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
    rayon_range_ev_cercle = 0.05;
    % Dessiner le carré
    rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
    rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
    viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

    % Calculer les coordonnées des coins du carré
    x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
    y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

    % Dessiner les cercles aux coins
    for i = 1:4
        % Tracer le cercle

        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
        plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
    end

   % Capturer l'image actuelle pour la vidéo
   frame = getframe(fig);
   writeVideo(video, frame);
   % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
   deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
   deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 end
 display('arrived to target');
 %init position evtol
 x_temp_1 = x_target_1; 
 y_temp_1 = y_target_1;

 x_temp_2 = x_target_2; 
 y_temp_2 = y_target_2;
 
 while(( Flag_arrived_base_1 == 0)&&( Flag_arrived_base_2 == 0))
    
    if (euclideanDistance(x_temp_1,y_temp_1,X_Emergency_1,Y_Emergency_1)> Seuil) 
       
        x_ev_trajectory1 = linspace(x_temp_1, X_Emergency_1, 50);
        y_ev_trajectory1 = linspace(y_temp_1, Y_Emergency_1, 50);
    
        x_temp_1 = x_temp_1 + (x_ev_trajectory1(2)-x_temp_1); 
        y_temp_1 = y_temp_1 + (y_ev_trajectory1(2)-y_temp_1); 
     
    else
        Flag_arrived_base_1 = 1;        
    end 
    
    if (euclideanDistance(x_temp_2,y_temp_2,X_Emergency_2,Y_Emergency_2)> Seuil)
       
        x_ev_trajectory2 = linspace(x_temp_2, X_Emergency_2, 50);
        y_ev_trajectory2 = linspace(y_temp_2, Y_Emergency_2, 50);
    
        x_temp_2 = x_temp_2 + (x_ev_trajectory2(2)-x_temp_2); 
        y_temp_2 = y_temp_2 + (y_ev_trajectory2(2)-y_temp_2);
    
    else
         Flag_arrived_base_2 = 1;   
    end
    
    if(euclideanDistance(x_temp_2,y_temp_2,x_temp_1,y_temp_1)< 2*rayon_evtol)
           display('obstacle');
        
       while(euclideanDistance(x_temp_2,y_temp_2,x_temp_1,y_temp_1)<2*rayon_evtol)
           display('dans while');
        
           if (euclideanDistance(x_temp_1,y_temp_1,X_Emergency_1,Y_Emergency_1)>(euclideanDistance(x_temp_2,y_temp_2,X_Emergency_2,Y_Emergency_2)))
                    display('dans if là');
                   [Min1,IdMin1] = SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);
                   display(IdMin1);
                   while ((Min1 <= 2*rayon_evtol)&&(euclideanDistance(x_temp_1,y_temp_1,X_Emergency_1,Y_Emergency_1)>euclideanDistance(x_temp_1,y_temp_1,x_temp_2,y_temp_2)))
                        display('dans while obs');
                        if (IdMin1==previus_IdMin)
                            cpt=cpt+1;
                        end
    
                       if (cpt ==80)
        
        
                        [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
                        while euclideanDistance(x_temp_1,y_temp_1,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
                                  % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                        y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                        ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré
                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                        y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                        ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré

                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Paramètres du carré et des cercles
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                        rayon_range_ev_cercle = 0.05;
                        % Dessiner le carré
                        rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                        rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                        viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Calculer les coordonnées des coins du carré
                        x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                        y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                        % Dessiner les cercles aux coins
                        for i = 1:4
                            % Tracer le cercle

                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                            plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                        end


                       % Capturer l'image actuelle pour la vidéo
                       frame = getframe(fig);
                       writeVideo(video, frame);
                       % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                       deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                       deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
                   
                       end
                        cpt=0; 
        
                       end
    
   
                        display(cpt);
                        switch IdMin1
        
                        case 1
                           x_temp_1 = x_temp_1 +0.01; 
                           y_temp_1 = y_temp_1 + 0.01;

                           previus_IdMin=1;
                           SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
                           break

                        case 2
                            x_temp_1 = x_temp_1 + 0.01; 
                            y_temp_1 = y_temp_1 -0.01;
                            previus_IdMin=2;
                            SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);

                            % Dessiner le nouveau carré de l'eVTOL

                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré
                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Dessiner le nouveau carré de l'eVTOL
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré

                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Paramètres du carré et des cercles
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                            rayon_range_ev_cercle = 0.05;
                            % Dessiner le carré
                            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Calculer les coordonnées des coins du carré
                            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                            % Dessiner les cercles aux coins
                            for i = 1:4
                                % Tracer le cercle

                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                            end


                           % Capturer l'image actuelle pour la vidéo
                           frame = getframe(fig);
                           writeVideo(video, frame);
                           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                           break;

                        case 3
                            x_temp_1 = x_temp_1 - 0.01; 
                            y_temp_1 = y_temp_1 + 0.01;
                            previus_IdMin=3;
                            SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);


                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                            break;

                        otherwise
                             x_temp_1 = x_temp_1 - 0.01; 
                             y_temp_1 = y_temp_1 - 0.01;
                             previus_IdMin=4;
                             SensorDetectionForObstacle(x_temp_1,y_temp_1,rayon_ev_carre,x_temp_2,y_temp_2);

                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                             break;
                        end                        
                    end
           else
                   display('dans ELSE');
                   [Min2,IdMin2] = SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);
                   while ((Min2 <= 2*rayon_evtol)&&(euclideanDistance(x_temp_2,y_temp_2,X_Emergency_2,Y_Emergency_2)>euclideanDistance(x_temp_2,y_temp_2,x_temp_1,y_temp_1)))
                        display('dans le while du else');
                        if (IdMin2==previus_IdMin_2)
                            cpt=cpt+1;
                        end
    
                        if (cpt ==80)
        
        
                        [ X_emergency_Waypoint,Y_emergency_Waypoint] = ObstaclePosition(0,2);
                        while euclideanDistance(x_temp,y_temp,X_emergency_Waypoint,Y_emergency_Waypoint)>0.4
            
                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                        y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                        ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré
                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Dessiner le nouveau carré de l'eVTOL
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                        y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                        ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Dessiner les cercles aux coins du nouveau carré

                        for j = 1:4
                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                            circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end

                        % Paramètres du carré et des cercles
                        rayon_ev_carre = 0.04; % Rayon du carré
                        rayon_ev_cercle = 0.02; % Rayon des cercles
                        centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                        rayon_range_ev_cercle = 0.05;
                        % Dessiner le carré
                        rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                        % Générer le cercle autour de l'eVTOL (sensor range)
                        centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                        rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                        viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                        % Calculer les coordonnées des coins du carré
                        x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                        y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                        % Dessiner les cercles aux coins
                        for i = 1:4
                            % Tracer le cercle

                            theta_circle = linspace(0, 2*pi, 100);
                            x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                            y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                            plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                        end

                       % Capturer l'image actuelle pour la vidéo
                       frame = getframe(fig);
                       writeVideo(video, frame);
                       % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                       deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                       deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
                   
                        end
                        cpt=0; 
        
                        end
    
   
                        display(cpt);
                        switch IdMin2
        
                        case 1
                           x_temp_2 = x_temp_2 +0.01; 
                           y_temp_2 = y_temp_2 + 0.01;

                           previus_IdMin_2=1;
                           SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end
                                
                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end


                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
                           break

                        case 2
                            x_temp_2 = x_temp_2 + 0.01; 
                            y_temp_2 = y_temp_2 -0.01;
                            previus_IdMin_2=2;
                            SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                            % Dessiner le nouveau carré de l'eVTOL

                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré
                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Dessiner le nouveau carré de l'eVTOL
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Dessiner les cercles aux coins du nouveau carré

                            for j = 1:4
                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                            end

                            % Paramètres du carré et des cercles
                            rayon_ev_carre = 0.04; % Rayon du carré
                            rayon_ev_cercle = 0.02; % Rayon des cercles
                            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                            rayon_range_ev_cercle = 0.05;
                            % Dessiner le carré
                            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                            % Générer le cercle autour de l'eVTOL (sensor range)
                            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                            % Calculer les coordonnées des coins du carré
                            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                            % Dessiner les cercles aux coins
                            for i = 1:4
                                % Tracer le cercle

                                theta_circle = linspace(0, 2*pi, 100);
                                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                            end

                           % Capturer l'image actuelle pour la vidéo
                           frame = getframe(fig);
                           writeVideo(video, frame);
                           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                           break;

                        case 3
                            x_temp_2 = x_temp_2 - 0.01; 
                            y_temp_2 = y_temp_2 + 0.01;
                            previus_IdMin_2=3;
                            SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);


                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                            break;

                        otherwise
                             x_temp_2 = x_temp_2 - 0.01; 
                             y_temp_2 = y_temp_2 - 0.01;
                             previus_IdMin_2=4;
                             SensorDetectionForObstacle(x_temp_2,y_temp_2,rayon_ev_carre,x_temp_1,y_temp_1);

                                 % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré
                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Dessiner le nouveau carré de l'eVTOL
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Dessiner les cercles aux coins du nouveau carré

                                for j = 1:4
                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                                end

                                % Paramètres du carré et des cercles
                                rayon_ev_carre = 0.04; % Rayon du carré
                                rayon_ev_cercle = 0.02; % Rayon des cercles
                                centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
                                rayon_range_ev_cercle = 0.05;
                                % Dessiner le carré
                                rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

                                % Générer le cercle autour de l'eVTOL (sensor range)
                                centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
                                rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
                                viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                                % Calculer les coordonnées des coins du carré
                                x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
                                y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

                                % Dessiner les cercles aux coins
                                for i = 1:4
                                    % Tracer le cercle

                                    theta_circle = linspace(0, 2*pi, 100);
                                    x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                                    y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                                    plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
                                end

                               % Capturer l'image actuelle pour la vidéo
                               frame = getframe(fig);
                               writeVideo(video, frame);
                               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
                               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
                               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);

                             break;
                        end                        
                    end
           end
     
                 % Dessiner le nouveau carré de l'eVTOL
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
                y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
                ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Dessiner les cercles aux coins du nouveau carré
                for j = 1:4
                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                    circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
                end

                % Dessiner le nouveau carré de l'eVTOL
                rayon_ev_carre = 0.04; % Rayon du carré
                rayon_ev_cercle = 0.02; % Rayon des cercles
                x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
                y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
                ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

                % Générer le cercle autour de l'eVTOL (sensor range)
                centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

                % Dessiner les cercles aux coins du nouveau carré

                for j = 1:4
                    theta_circle = linspace(0, 2*pi, 100);
                    x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                    y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                    circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
                end


               % Capturer l'image actuelle pour la vidéo
               frame = getframe(fig);
               writeVideo(video, frame);
               % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
               deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
               deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 
        end  

    else
            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
            y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
            ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré
            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Dessiner le nouveau carré de l'eVTOL
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
            y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
            ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Dessiner les cercles aux coins du nouveau carré

            for j = 1:4
                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
                circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
            end

            % Paramètres du carré et des cercles
            rayon_ev_carre = 0.04; % Rayon du carré
            rayon_ev_cercle = 0.02; % Rayon des cercles
            centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
            rayon_range_ev_cercle = 0.05;
            % Dessiner le carré
            rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

            % Générer le cercle autour de l'eVTOL (sensor range)
            centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
            rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
            viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

            % Calculer les coordonnées des coins du carré
            x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
            y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

            % Dessiner les cercles aux coins
            for i = 1:4
                % Tracer le cercle

                theta_circle = linspace(0, 2*pi, 100);
                x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
                y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
                plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
            end

           % Capturer l'image actuelle pour la vidéo
           frame = getframe(fig);
           writeVideo(video, frame);
           % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
           deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
           deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
            
               
    end
    
     % Dessiner le nouveau carré de l'eVTOL
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    x_corners = [x_temp_1 - rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 + rayon_ev_carre, x_temp_1 - rayon_ev_carre];
    y_corners = [y_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, y_temp_1 + rayon_ev_carre, y_temp_1 + rayon_ev_carre];
    ev_handle_1 = rectangle('Position', [x_temp_1 - rayon_ev_carre, y_temp_1 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'r', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [x_temp_1,y_temp_1]; % Coordonnées du centre de l'eVTOL
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ev_sensor_range_1=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
    
    % Dessiner les cercles aux coins du nouveau carré
    for j = 1:4
        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
        circle_handles_1(j) = plot(x_circle_corner, y_circle_corner, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
    
    % Dessiner le nouveau carré de l'eVTOL
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    x_corners = [x_temp_2 - rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 + rayon_ev_carre, x_temp_2 - rayon_ev_carre];
    y_corners = [y_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, y_temp_2 + rayon_ev_carre, y_temp_2 + rayon_ev_carre];
    ev_handle_2 = rectangle('Position', [x_temp_2 - rayon_ev_carre, y_temp_2 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'b', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [x_temp_2,y_temp_2]; % Coordonnées du centre de l'eVTOL
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rayon_evtol = ValSensorRange; % Rayon du cercle autour de l'eVTOL    C EST IIIIIIIIIIIIIIIIIICIIIIIIIIIIII
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ev_sensor_range_2=viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
    
    % Dessiner les cercles aux coins du nouveau carré

    for j = 1:4
        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(j) + rayon_ev_cercle * sin(theta_circle);
        circle_handles_2(j) = plot(x_circle_corner, y_circle_corner, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');
    end

    % Paramètres du carré et des cercles
    rayon_ev_carre = 0.04; % Rayon du carré
    rayon_ev_cercle = 0.02; % Rayon des cercles
    centre_range_ev_cercle= [0.4,y_evtol_3 - rayon_ev_carre];
    rayon_range_ev_cercle = 0.05;
    % Dessiner le carré
    rectangle('Position', [x_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, 2*rayon_ev_carre, 2*rayon_ev_carre], 'EdgeColor', 'k', 'LineWidth', 2);

    % Générer le cercle autour de l'eVTOL (sensor range)
    centre_evtol = [0.4, y_evtol_3]; % Coordonnées du centre de l'eVTOL
    rayon_evtol = 0.2; % Rayon du cercle autour de l'eVTOL
    viscircles(centre_evtol, rayon_evtol, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);

    % Calculer les coordonnées des coins du carré
    x_corners = [x_evtol_3 - rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 + rayon_ev_carre, x_evtol_3 - rayon_ev_carre];
    y_corners = [y_evtol_3 - rayon_ev_carre, y_evtol_3 - rayon_ev_carre, y_evtol_3 + rayon_ev_carre, y_evtol_3 + rayon_ev_carre];

    % Dessiner les cercles aux coins
    for i = 1:4
        % Tracer le cercle

        theta_circle = linspace(0, 2*pi, 100);
        x_circle_corner = x_corners(i) + rayon_ev_cercle * cos(theta_circle);
        y_circle_corner = y_corners(i) + rayon_ev_cercle * sin(theta_circle);
        plot(x_circle_corner, y_circle_corner, 'k', 'LineWidth', 2);
    end

   % Capturer l'image actuelle pour la vidéo
   frame = getframe(fig);
   writeVideo(video, frame);
   % Définir la fonction pour supprimer l'eVTOL précédent et les cercles associés
   deletAll(ev_handle_1,circle_handles_1,ev_sensor_range_1);
   deletAll(ev_handle_2,circle_handles_2,ev_sensor_range_2);
 end