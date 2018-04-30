% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Script para teste e depuracao do algoritmo A estrela

%% Struct do No
% node.x = x
% node.y = y
% node.g = INF (valor default = infinito)
% node.f = INF (valor default = infinito)
% node.camefrom = NULL

clear

% Valor 'infinito'
INF = 100000;
% Limites do grid de celulas
GRID_MIN = 1;
GRID_MAX = 5;

grid_ = [1 1 1 1 1;
        1 1 1 1 1;
        1 1 1 1 1;
        1 1 1 1 1;
        1 1 1 1 1];

start_node.x = 1;
start_node.y = 1;
start_node.g = INF;
start_node.f = INF;
start_node.camefrom = [];

goal_node.x = 3;
goal_node.y = 4;
goal_node.g = INF;
goal_node.f = INF;
goal_node.camefrom = [];

path_nodes = a_star(start_node, goal_node);

x = [];
y = [];

for i = path_nodes
   x = [x i.x];
   y = [y i.y];
end

x(1) = [];
x(end) = [];

y(1) = [];
y(end) = [];

axis([GRID_MIN-1 GRID_MAX+1 GRID_MIN-1 GRID_MAX+1])
grid on
hold on
stem(start_node.x, start_node.y, 'filled', 'LineStyle', 'none', 'MarkerFaceColor','green')
stem(goal_node.x, goal_node.y, 'filled', 'LineStyle', 'none', 'MarkerFaceColor','red')
stem(x, y, 'filled', 'LineStyle', 'none', 'MarkerFaceColor','blue')

