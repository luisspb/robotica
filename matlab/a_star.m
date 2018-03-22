% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Algoritmo A*

%% Estrutura do No
% node.x = 1;
% node.y = 1;
% node.g = 1;
% node.f = 8;
% node.parent = 0;

function nodes_path = a_star(start_node, goal_node)

   closed_set = [];
   open_set = [start_node];
   path = [];

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

      %% Lista de vizinhos
      % Vizinho de cima
      node_up = current_node;
      node_up.y = node_up.y + 1;
      % Vizinho de baixo
      node_down = current_node;
      node_down.y = node_down.y - 1;
      % Vizinho da esquerda
      node_left = current_node;
      node_left.x = node_left.x - 1;
      % Vizinho da direita
      node_right = current_node;
      node_right.x = node_right.x + 1;
      % Cria a lista de vizinhos
      neighbor_nodes = [node_up node_down node_left node_right];

      for neighbor = neighbor_nodes

         

      end



   end

end


