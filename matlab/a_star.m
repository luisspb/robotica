% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Algoritmo A*

%% No inicial
% start_node.x = 1;
% start_node.y = 1;
% start_node.g = ;
% start_node.f = ;

%% No final
% goal_node.x = 9;
% goal_node.y = 9;
% goal_node.g = ;
% goal_node.f = 0;

function nodes_path = a_star(start_node, goal_node)

   closed_set = [];
   open_set = [start_node];
   came_from = [];

   start_node.g = 0;
   start_node.f = start_node.g + manhattan_distance(start_node, goal_node);

   while ~isempty(open_set)

      current_node = min_f(open_set);
      nodes_path = [nodes_path current_node];

      % Remove current_node de open_set
      n = 1;
      for i = open_set
         if isequaln(i, current_node)
            open_set(n) = [];
         else
            n = n + 1;
         end
      end

      % Adiciona current_node em closed_set
      closed_set = [closed_set current_node];

      % Cria lista de vizinhos
      node = current_node;
      node = current_node.x
      neighbor_nodes = 

   end

end


