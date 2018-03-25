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

% Valor 'infinito'
INF = 100000;
% Limites do grid de celulas
GRID_MIN = 1;
GRID_MAX = 5;

grid = [1 1 1 1 1;
        1 1 1 1 1;
        1 1 1 1 1;
        1 1 1 1 1;
        1 1 1 1 1];

start_node.x = 1;
start_node.y = 1;

goal_node.x = 4;
goal_node.y = 3;

path_nodes = a_star(start_node, goal_node);

