function MAP = generate_random_map(density)
    % Vérification de la densité (0 < density <= 1)
    if density <= 0 || density > 1
        error('La densité doit être comprise entre 0 (exclus) et 1 (inclus).');
    end
    
    % Initialisation d'une matrice remplie de zéros
    MAP = int8(zeros(128, 140));
    
    % Génération aléatoire des obstacles
    for i = 1:128
        for j = 1:140
            if rand() < density
                MAP(i, j) = 1; % Place un obstacle
            end
        end
    end
end