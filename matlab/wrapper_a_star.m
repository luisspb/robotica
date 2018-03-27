% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Funcao wrapper que insere o algoritmo do A estrela dentro do modelo do
% robo

function [x, y] = wrapper_a_star(grid, start, goal)

   % Valor 'infinito'
   INF = 100000;
   % Limites do grid de celulas
   GRID_MIN = 1;
   GRID_MAX = 5;

   start_node.x = start(1);
   start_node.y = start(2);
   start_node.g = INF;
   start_node.f = INF;
   start_node.camefrom = [];

   goal_node.x = goal(1);
   goal_node.y = goal(2);
   goal_node.g = INF;
   goal_node.f = INF;
   goal_node.camefrom = [];

   path_nodes = a_star(start_node, goal_node);

   for i = path_nodes
      x = i.x;
      y = i.y;
   end
end
