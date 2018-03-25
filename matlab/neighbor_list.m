% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Funcao que cria lista de vizinhos de um no
% Por enquanto nao trata nos (celulas) bloqueadas

function neighbor_nodes = neighbor_list(current_node)

   % Valor 'infinito'
   INF = 100000;
   % Limites do grid de celulas
   GRID_MIN = 1;
   GRID_MAX = 5;

   neighbor_nodes = [];

   % Vizinho de cima
   node_up = current_node;
   node_up.y = node_up.y + 1;
   % is_blocked? Adicionar depois!
   if node_up.y <= GRID_MAX
      node_up.g = INF;
      node_up.f = INF;
      node_up.camefrom = current_node;
      neighbor_nodes = [neighbor_nodes node_up];
   end

   % Vizinho de baixo
   node_down = current_node;
   node_down.y = node_down.y - 1;
   % is_blocked? Adicionar depois!
   if node_down.y >= GRID_MIN
      node_down.g = INF;
      node_down.f = INF;
      node_down.camefrom = current_node;
      neighbor_nodes = [neighbor_nodes node_down];
   end

   % Vizinho da esquerda
   node_left = current_node;
   node_left.x = node_left.x - 1;
   % is_blocked? Adicionar depois!
   if node_left.x >= GRID_MIN
      node_left.g = INF;
      node_left.f = INF;
      node_left.camefrom = current_node;
      neighbor_nodes = [neighbor_nodes node_left];
   end

   % Vizinho da direita
   node_right = current_node;
   node_right.x = node_right.x + 1;
   % is_blocked? Adicionar depois!
   if node_right.x <= GRID_MAX
      node_right.g = INF;
      node_right.f = INF;
      node_right.camefrom = current_node;
      neighbor_nodes = [neighbor_nodes node_right];
   end

end

