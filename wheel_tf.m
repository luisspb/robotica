% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Definicao da Funcao de Transferencia do motor da roda do TurtleBot 

%% Parametros da funcao
n = 0.99;
R = 1.5506;
kt = 10.913e-3;
kv = 11.518e-3; kem = kv;
Je = 9.35604e-3;

%% Funcao de transferencia

% Numerador
num = [n*kt]

% Denominador
den = [kem*kt R*Je]

% Definicao da funcao de transferencia do motor da roda
wheel = tf(num, den)

% Resposta ao degrau em malha fechada
wheel_cl = feedback(wheel, 1)  % wheel_cl = FT do motor da roda em 'closed loop'
figure(1)
step(wheel_cl, 0.1)

% Zeros e Polos da funcao
zero(wheel)
pole(wheel)

% Plot dos Zeros e Polos
figure(2)
pzplot(wheel)

% Funcao de transferencia com Zeros e Polos em evidencia
wheel_zp = zpk(wheel)  % wheel_zp = FT do motor da roda com (z) zeros e (p) polos

