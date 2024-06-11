function [randomPosX, randomPosY] = RandomPositionObstacleGenerator(X_originPosObstacle,Y_originPosObstacle,rayon_evtol)



randomPosX=X_originPosObstacle;
randomPosY=Y_originPosObstacle;
ListePathNodeObstacle = zeros(10,2);

for i = 1 : size(ListePathNodeObstacle)
    
    ListePathNodeObstacle(i,1) = ObstaclePosition(rayon_evtol,(2-rayon_evtol));
    ListePathNodeObstacle(i,2) = ObstaclePosition(rayon_evtol,(2-rayon_evtol));
end

for i = 1:size(ListePathNodeObstacle)

while euclideanDistance(randomPosX,randomPosY,X_Emergency,Y_Emergency)> Seuil
    
    
end






end