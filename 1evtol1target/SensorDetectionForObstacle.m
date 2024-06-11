   
function [Min1,IdMin1] = SensorDetectionForObstacle(x_temp,y_temp,rayon_ev_carre,X_Obstacle_to_consider,Y_Obstacle_to_consider)

    sensorBottomLeftO1 = euclideanDistance(x_temp - rayon_ev_carre,y_temp - rayon_ev_carre,X_Obstacle_to_consider,Y_Obstacle_to_consider);
    sensorTopLeftO1 = euclideanDistance(x_temp - rayon_ev_carre,y_temp + rayon_ev_carre,X_Obstacle_to_consider,Y_Obstacle_to_consider);
    sensorBottomRightO1 = euclideanDistance(x_temp + rayon_ev_carre,y_temp - rayon_ev_carre,X_Obstacle_to_consider,Y_Obstacle_to_consider);
    sensorTopRightO1 = euclideanDistance(x_temp + rayon_ev_carre,y_temp + rayon_ev_carre,X_Obstacle_to_consider,Y_Obstacle_to_consider);
        
    ListeSensorO1 = [sensorBottomLeftO1,sensorTopLeftO1,sensorBottomRightO1,sensorTopRightO1];
   
    [Min1,IdMin1] = min(ListeSensorO1);
end