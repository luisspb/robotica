%Xt1 posi��o inicial: x, y e o angulo de orienta��o teta [x; y; teta]
%Ut � um vetor com as velocidades linear e angular [v; w]

Xt1 = [0; 0; 0];
Ut = [1;0];
%alfa = [2 2 2 2 2 2] * 1e-4;
alfa = [2 2 2 2 2 2] * 1e-2;
tempo = 0; 
fazer_quadrado = 0;

while(tempo < 6)
    
    X_inicioEfim = [];    % Vetor para as coordenadas x das posi��es inicial e final
    Y_inicioEfim = [];    % Vetor para as coordenadas y das posi��es inicial e final
    X_inicioEfim(1) = Xt1(1);
    Y_inicioEfim(1) = Xt1(2);
    x = []; y=[];
    x = Xt1(1);     % x � um vetor das coordenadas x de todas as posi��es preditas
    y = Xt1(2);     % y � um vetor das coordenadas y de todas as posi��es preditas
    tetas = [];
    for i = 1:500  % 1000 � a quantidade de amostra
        Xt = sample_motion_model_velocity(Ut, Xt1, tempo, alfa);        
        x=[x Xt(1)];
        y=[y Xt(2)];
        
    end
    
    X_inicioEfim(2) = Xt(1);
    Y_inicioEfim(2) = Xt(2);
    
    if(fazer_quadrado == 1)
        if(mod(tempo,2)==0 && tempo ~= 0) Xt(3) = Xt(3) + 90; end
    end
    
    Xt1 = Xt;
    
    plot(x,y,'r.', X_inicioEfim,Y_inicioEfim, 'b-O')
    hold on
    grid on
    legend('Samples');
    
    if(fazer_quadrado ~= 1) ylim([-2 8]); end

    tempo = tempo + 1;
end

