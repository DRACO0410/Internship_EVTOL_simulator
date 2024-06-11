function [centre_evtol,x_circle_corner,y_circle_corner]= calculateNewEvtolParam(x_temp,y_temp,rayon_ev_carre,rayon_ev_cercle)

      x_corners = [x_temp - rayon_ev_carre, x_temp + rayon_ev_carre, x_temp + rayon_ev_carre, x_temp - rayon_ev_carre];
      y_corners = [y_temp - rayon_ev_carre, y_temp - rayon_ev_carre, y_temp + rayon_ev_carre, y_temp + rayon_ev_carre];
      centre_evtol = [x_temp,y_temp]; % Coordonnées du centre de l'eVTOL
      
      
      x_circle_corner= [];
      y_circle_corner = [];
      for j = 1:4
       theta_circle = linspace(0, 2*pi, 100);
       x_circle_corner = x_corners(j) + rayon_ev_cercle * cos(theta_circle);
       y_circle_corner = y_corners(j) + rayon_ev_cercle * cos(theta_circle);
      end
end