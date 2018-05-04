%Xt1 posição inicial: x, y e o angulo de orientação teta [x y teta]
%Ut é um vetor com as velocidades linear e angular [v w]

Xt1 = [0 0 30];
Ut = [1 0];

X_inicioEfim = [];    % Vetor para as coordenadas x das posições inicial e final
Y_inicioEfim = [];    % Vetor para as coordenadas y das posições inicial e final
X_inicioEfim(1) = Xt1(1);
Y_inicioEfim(1) = Xt1(2);

x = Xt1(1);     % x é um vetor das coordenadas x de todas as posições preditas
y = Xt1(2);     % y é um vetor das coordenadas y de todas as posições preditas

for i = 1:1000  % 1000 é a quantidade de amostra
    Xt = sample_motion_model_velocity(Ut, Xt1);
    x=[x, Xt(1)];
    y=[y, Xt(2)];
    X_inicioEfim(2) = Xt(1);
    Y_inicioEfim(2) = Xt(2);
end
plot(x,y,'r.', X_inicioEfim,Y_inicioEfim, 'b-O')
