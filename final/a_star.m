% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Algoritmo A*

%% Struct do No
% node.x = x
% node.y = y
% node.g = INF (valor default = infinito)
% node.f = INF (valor default = infinito)
% node.camefrom = EMPTY

function path_nodes = a_star(start_node, goal_node, gridSize)

   % Valor 'infinito'
   INF = 100000;
   % Limites do grid de celulas
   grid_x = [0 gridSize(1)-1];
   grid_y = [0 gridSize(2)-1];

   start_node.camefrom = [];
   start_node.g = 0;
   start_node.f = start_node.g + manhattan_distance(start_node, goal_node);

   closed_set = [];
   open_set = [start_node];
   path_nodes = [];

   while ~isempty(open_set)

      current_node = min_f(open_set);

      if (current_node.x == goal_node.x) && (current_node.y == goal_node.y)
         path_nodes = reconstruct_path(current_node);
         return
      end

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

      % Lista de vizinhos
      neighbor_nodes = neighbor_list(current_node, grid_x, grid_y);

      for neighbor = neighbor_nodes

         % Procura se o neighbor esta no closed_set,
         % se estiver pula essa iteracao, no ja foi avaliado
         is_in_closed_set = 0;
         for i = closed_set
            if (neighbor.x == i.x && neighbor.y == i.y)
               is_in_closed_set = 1;
            end
         end
         if is_in_closed_set
            continue
         end

         % Procura se o neighbor esta no open_set,
         % se nao estiver, o adiciona na lista aberta
         is_in_open_set = 0;
         for i = open_set
            if (neighbor.x == i.x && neighbor.y == i.y)
               is_in_open_set = 1;
            end
         end
         if ~is_in_open_set
            neighbor.g = current_node.g + 1;
            neighbor.f = current_node.g + manhattan_distance(current_node, goal_node);
            open_set = [open_set neighbor];
         end

         % Compara o custo previo do vizinho com o custo do caminho atual.
         % Se o custo atual for menor do que o custo ja armazenado no vizinho,
         % foi encontrado um melhor caminho e deve registra-lo
         tentative_g = current_node.g + distance_between(current_node, neighbor);
         if tentative_g < neighbor.g
            neighbor.camefrom = current_node;
            neighbor.g = tentative_g;
         end

      end  % for neighbor = neighbor_nodes

   end  % while ~isempty(open_set)
   % Retorna falha, caso a lista aberta fique vazia antes de se atingir o alvo
   path_nodes = [];
   return

end  % a_star function
