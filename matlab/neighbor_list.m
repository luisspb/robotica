% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Funcao que cria lista de vizinhos de um no
% Por enquanto nao trata nos (celulas) bloqueadas

function neighbor_nodes = neighbor_list(current_node)

      % Vizinho de cima
      node_up = current_node;
      node_up.y = node_up.y + 1;
      % is_blocked? Adicionar depois!
      node_up.g = INF;
      node_up.f = INF;
      node_up.parent = current_node;

      % Vizinho de baixo
      node_down = current_node;
      node_down.y = node_down.y - 1;
      % is_blocked? Adicionar depois!
      node_up.g = INF;
      node_up.f = INF;
      node_up.parent = current_node;

      % Vizinho da esquerda
      node_left = current_node;
      node_left.x = node_left.x - 1;
      % is_blocked? Adicionar depois!
      node_up.g = INF;
      node_up.f = INF;
      node_up.parent = current_node;

      % Vizinho da direita
      node_right = current_node;
      node_right.x = node_right.x + 1;
      % is_blocked? Adicionar depois!
      node_up.g = INF;
      node_up.f = INF;
      node_up.parent = current_node;

      % Cria a lista de vizinhos
      neighbor_nodes = [node_up node_down node_left node_right];
end

