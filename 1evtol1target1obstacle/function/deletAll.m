function deletAll(ev_handle,circle_handles,ev_sensor_range,text_handles)

    delete_previous_eVTOL = @(h) delete(h);
    delete_previous_circles = @(h) delete(h);
    delete_previous_text = @(h) delete(h);
    
    % Supprimer l'eVTOL précédent et les cercles associés
    delete_previous_eVTOL(ev_handle);
    delete_previous_circles(circle_handles); 
    delete_previous_circles(ev_sensor_range);
    delete_previous_text(text_handles);
    

end 