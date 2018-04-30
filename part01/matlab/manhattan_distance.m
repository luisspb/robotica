% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Heuristica - Distancia de Manhattan

function distance = manhattan_distance(start_node, goal_node)

   distance = abs(goal_node.x - start_node.x);
   distance = distance + abs(goal_node.y - start_node.y);

end

