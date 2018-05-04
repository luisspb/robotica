%Xt1 posi��o inicial: x, y e o angulo de orienta��o teta [x y teta]
%Ut � um vetor com as velocidades linear e angular [v w]

Xt1 = [0 0 0];
Ut = [1 0];

X_inicioEfim = [];    % Vetor para as coordenadas x das posi��es inicial e final
Y_inicioEfim = [];    % Vetor para as coordenadas y das posi��es inicial e final
X_inicioEfim(1) = Xt1(1);
Y_inicioEfim(1) = Xt1(2);

x = Xt1(1);     % x � um vetor das coordenadas x de todas as posi��es preditas
y = Xt1(2);     % y � um vetor das coordenadas y de todas as posi��es preditas

for i = 1:1000  % 1000 � a quantidade de amostra
    Xt = sample_motion_model_velocity(Ut, Xt1);
    x=[x, Xt(1)];
    y=[y, Xt(2)];
    X_inicioEfim(2) = Xt(1);
    Y_inicioEfim(2) = Xt(2);
end
plot(x,y,'r+', X_inicioEfim,Y_inicioEfim, 'b-O')