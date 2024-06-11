function [x_obstacle1,y_obstacle1]= ObstaclePosition (a,b)
        
            % function which permit to initialize obstacle coordonates
            
            random_posX = a + (b-a) * rand(1); 
            random_posY = a + (b-a) * rand(1); 
            x_obstacle1 = random_posX; 
            y_obstacle1 = random_posY;           
        end