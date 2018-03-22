% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Algoritmo A*

INF = 100000;

%% Struct do No
% node.x = x
% node.y = y
% node.g = INF (valor default = infinito)
% node.f = INF (valor default = infinito)
% node.camefrom = NULL

function path_nodes = a_star(start_node, goal_node)

   closed_set = [];
   open_set = [start_node];
   path_nodes = [];

   start_node.g = 0;
   start_node.f = start_node.g + manhattan_distance(start_node, goal_node);

   while ~isempty(open_set)

      current_node = min_f(open_set);

      if isequaln(current_node, goal_node)
         return = reconstruct_path(current_node);

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
      neighbor_nodes = neighbor_list(current_node);

      for neighbor = neighbor_nodes

         % Procura se o neighbor esta no closed_set,
         % se estiver pula essa iteracao, no ja foi avaliado
         is_in_closed_set = 0;
         for i = closed_set
            if isequaln(neighbor, i)
               is_in_closed_set = 1;
            end
         end
         if is_in_closed_set == 1
            continue
         end

         % Procura se o neighbor esta no open_set,
         % se nao estiver, o adiciona na lista aberta
         is_in_open_set = 0;
         for i = open_set
            if isequaln(neighbor, i)
               is_in_open_set = 1;
            end
         end
         if is_in_open_set == 0
            open_set = [open_set neighbor]
         end

      end

   end
   % Retorna falha, caso a lista aberta fique vazia antes de se atingir o alvo
   return [];

end

