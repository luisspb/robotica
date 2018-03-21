% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Funcao que retorna o no com menor custo

function f = min_f(nodes)

   f = 10000;   % f inicial bem grande

   for i = nodes
      if f > i.f
         f = i.f;
      end
   end

end

