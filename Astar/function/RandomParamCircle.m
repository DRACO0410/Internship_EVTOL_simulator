
function [CentreX,CentreY,CarreRayon] = RandomParamCircle()

%generate rando parametre for obsacle

borne_inf_x = 1;
borne_sup_x = 140;

borne_inf_y = 1;
borne_sup_y = 128;

borne_inf_Rayon = 50;
borne_sup_Rayon = 300;

CentreX = randi([borne_inf_x,borne_sup_x]);
CentreY = randi([borne_inf_y,borne_sup_y]);
CarreRayon = randi([borne_inf_Rayon,borne_sup_Rayon]);
end
