% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Funcao que retorna o no com menor custo

function node = min_f(nodes)

   node = nodes(1);

   for i = nodes(2:end)
      if node.f > i.f
         node = i;
      end
   end

end

