% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Funcao que constroi o caminho planejado pelo algoritmo A estrela

function path_nodes = rescontruct_path(node)

   path_nodes = [];
   path_nodes = [path_nodes node];

   while node.camefrom
      node = node.camefrom;
      path_nodes = [path_nodes node];
   end

   path_nodes = fliplr(path_nodes);

end

