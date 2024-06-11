%Example on the use of AStar Algorithm in an occupancy grid. 
clear


%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
[CentreX,CentreY,CarreRayon]=RandomParamCircle();
[CentreX1,CentreY1,CarreRayon1]=RandomParamCircle();
[CentreX2,CentreY2,CarreRayon2]=RandomParamCircle();
[CentreX3,CentreY3,CarreRayon3]=RandomParamCircle();
[CentreX4,CentreY4,CarreRayon4]=RandomParamCircle();
[CentreX5,CentreY5,CarreRayon5]=RandomParamCircle();
[CentreX6,CentreY6,CarreRayon6]=RandomParamCircle();
[CentreX7,CentreY7,CarreRayon7]=RandomParamCircle();
[CentreX8,CentreY8,CarreRayon8]=RandomParamCircle();
[CentreX9,CentreY9,CarreRayon9]=RandomParamCircle();
[CentreX10,CentreY10,CarreRayon10]=RandomParamCircle();


%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
MAP=int8(zeros(128,140));

for i = 1:128
    for j = 1:140
        
        
        MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX,CentreY,CarreRayon,MAP(i,j));
        MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX1,CentreY1,CarreRayon1,MAP(i,j));
%         MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX2,CentreY2,CarreRayon2,MAP(i,j));
%         MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX3,CentreY3,CarreRayon3,MAP(i,j));
%         MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX4,CentreY4,CarreRayon4,MAP(i,j));
%         MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX5,CentreY5,CarreRayon5,MAP(i,j));
%         MAP(i,j) =  MAP(i,j) + CreateRandomeCircle(i,j,CentreX6,CentreY6,CarreRayon6,MAP(i,j));
       

        
        
    end
end


%Start Positions
StartX=20;
StartY=100;
% Génération aléatoire des coordonnées de la cible en danger
% Tracé de la droite verticale orange
x_vertical_line = [20 20]; % Coordonnées en x
y_vertical_line = [95  105]; % Coordonnées en y

x_target1=int8(ObstaclePosition(70,139));
y_target1 = int8(ObstaclePosition(1,127));

   
X_Emergency=20;
Y_Emergency=40;
EmergencyGoal= int8([Y_Emergency,X_Emergency]);

%Generating goal nodes, which is represented by a matrix. Several goals can be speciefied, in which case the pathfinder will find the closest goal. 
%a cell with the value 1 represent a goal cell
GoalRegister=int8(zeros(128,140));
GoalRegister(y_target1,x_target1)=1;

GoalRegisterEMER=int8(zeros(128,140));
GoalRegisterEMER(Y_Emergency,X_Emergency)=1;


%Number of Neighboors one wants to investigate from each cell. A larger
%number of nodes means that the path can be alligned in more directions. 
%Connecting_Distance=1-> Path can  be alligned along 8 different direction.
%Connecting_Distance=2-> Path can be alligned along 16 different direction.
%Connecting_Distance=3-> Path can be alligned along 32 different direction.
%Connecting_Distance=4-> Path can be alligned along 56 different direction.
%ETC......

Connecting_Distance=8; %Avoid to high values Connecting_Distances for reasonable runtimes. 

% Running PathFinder
OptimalPathToTarget=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance);
OptimalPathToEmergency=ASTARPATH(x_target1,y_target1,MAP,GoalRegisterEMER,Connecting_Distance);

% End. 

if ((size(OptimalPathToTarget,2)>1)&&(size(OptimalPathToEmergency,2)>1))
figure(10)
imagesc((MAP))
    colormap(flipud(gray));


hold on

x_danger = [70 70 140 140]; % Coordonnées en x du danger zone (km)
y_danger = [0 128 128 0]; % Coordonnées en y du danger zone (km)

% Tracé du Danger zone
plot(x_danger, y_danger, 'r', 'LineWidth', 2);

% Tracé du trait vertical avec un style de ligne en pointillés et une couleur grise
plot([70 70], [0 128], 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Imaginary Boundary');
plot(x_vertical_line, y_vertical_line, 'Color', [1, 0.5, 0], 'LineWidth', 2, 'DisplayName', 'Take off zone');


x_vertiport = [70 0 0 70]; % Coordonnées en x du vertiport (km)
y_vertiport = [128 128 90 90]; % Coordonnées en y du vertiport (km)

% Tracé du Vertiport
plot(x_vertiport, y_vertiport, 'g','LineWidth',2);
%trcé de la barre orange 
plot(x_vertical_line, y_vertical_line, 'Color', [1, 0.5, 0], 'LineWidth', 2, 'DisplayName', 'Take off zone');
%tracé du point emergency
plot(X_Emergency, Y_Emergency, 'bo', 'MarkerSize', 4, 'LineWidth',8, 'DisplayName', 'Emergency');
%tracé de la croix représentant la target 
plot([x_target1-5 x_target1+5], [y_target1 y_target1], 'k', 'LineWidth', 2,'DisplayName','test'); % Tracé de la ligne horizontale
plot([x_target1 x_target1], [y_target1-5 y_target1+5], 'k', 'LineWidth', 2,'DisplayName','test'); % Tracé de la ligne verticale


%tracé du chemin
plot(OptimalPathToTarget(end,2),OptimalPathToTarget(end,1),'o','color','b','DisplayName','Starting Point');
plot(OptimalPathToTarget(:,2),OptimalPathToTarget(:,1),'r','DisplayName', 'Path to target');
plot(OptimalPathToEmergency(:,2),OptimalPathToEmergency(:,1),'g','DisplayName', 'Path to Emergency');


legend('Take off zone','Target','Starting Point','Path to target','Path to Emergency');


else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end









showNeighboors=0; %Set to 1 if you want to visualize how the possible directions of path. The code
%below are purley for illustrating purposes. 
if showNeighboors==1
%



%2
NeigboorCheck=[0 1 0 1 0;1 1 1 1 1;0 1 0 1 0;1 1 1 1 1;0 1 0 1 0]; %Heading has 16 possible allignments
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(2+1);
figure(2)

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
 
grid on
title('Connecting distance=2')
end

%3
NeigboorCheck=[0 1 1 0 1 1 0;1 0 1 0 1 0 1;1 1 1 1 1 1 1;0 0 1 0 1 0 0;1 1 1 1 1 1 1;1 0 1 0 1 0 1;0 1 1 0 1 1 0]; %Heading has 32 possible allignments
figure(3)
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(3+1);

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
 grid on
title('Connecting distance=3')

end
 
%4
NeigboorCheck=[0 1 1 1 0 1 1 1 0;1 0 1 1 0 1 1 0 1;1 1 0 1 0 1 0 1 1;1 1 1 1 1 1 1 1 1;0 0 0 1 0 1 0 0 0;1 1 1 1 1 1 1 1 1;1 1 0 1 0 1 0 1 1 ;1 0 1 1 0 1 1 0 1 ;0 1 1 1 0 1 1 1 0];  %Heading has 56 possible allignments
figure(4)
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(4+1);

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
grid on
title('Connecting distance=4')

end
%1
NeigboorCheck=[1 1 1;1 0 1;1 1 1];
figure(1)
[row col]=find(NeigboorCheck==1);
Neighboors=[row col]-(1+1);

for p=1:size(Neighboors,1)
  i=Neighboors(p,1);
       j=Neighboors(p,2);
      
     plot([0 i],[0 j],'k')
 hold on
 axis equal
grid on
title('Connecting distance=1')

end
end


