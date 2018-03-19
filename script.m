% Autores: Jorgeluis Guerra
%          Luis Braga
%          Saulo Alves

% Modelo do Robo

%% Parametros do Robo
vmax = 0.65;
b = 0.23;

%% Parametros da funcao de transferencia
n = 0.99;
R = 1.5506;
kt = 10.913e-3;
kv = 11.518e-3; kem = kv;
Je = 9.35604e-3;

%% Funcao de transferencia

% Numerador
num = [n*kt];

% Denominador
den = [R*Je kem*kt];

% Definicao da funcao de transferencia do motor da roda
wheel = tf(num, den);

% Resposta ao degrau em malha fechada
wheel_cl = feedback(wheel, 1);  % wheel_cl = FT do motor da roda em 'closed loop'
%figure(1)
%step(wheel_cl, 10)

% Plot dos Zeros e Polos em malha fechada
%figure(2)
%pzplot(wheel_cl)

% Zeros e Polos da funcao em malha aberta
zero(wheel);
pole(wheel);

% Funcao de transferencia em malha fechada com Zeros e Polos em evidencia
wheel_zp = zpk(wheel);  % wheel_zp = FT do motor da roda com (z) zeros e (p) polos

% Constantes do controlador PI
Kcr = 2.4;
Pcr = 27.7;
Ti = (1/1.2)*Pcr;
P = 0.45*Kcr
I = 1 / Ti

