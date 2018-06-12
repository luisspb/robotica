% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

%% Struct do No
% node.x = x
% node.y = y
% node.g = INF (valor default = infinito)
% node.f = INF (valor default = infinito)
% node.camefrom = NULL

function waypoints = wrapper_a_star(start, goal, gridSize)

  % Valor 'infinito'
  INF = 100000;

  % grid_ = [1 1 1 1 1;
  %         1 1 1 1 1;
  %         1 1 1 1 1;
  %         1 1 1 1 1;
  %         1 1 1 1 1];

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

  path_nodes = a_star(start_node, goal_node, gridSize);

  waypoints = [];

  for i = path_nodes
     waypoints = [waypoints; i.x i.y];
  end

end
