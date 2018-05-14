%Xt1 posição inicial: x, y e o angulo de orientação teta [x; y; teta]
%Ut é um vetor com as velocidades linear e angular [v; w]

Xt1 = [0; 0; 0];
Ut = [1;0];
%alfa = [2 2 2 2 2 2] * 1e-4;
alfa = [2 2 2 2 2 2] * 1e-2;
tempo = 0; 
fazer_quadrado = 0;

while(tempo < 6)
    
    X_inicioEfim = [];    % Vetor para as coordenadas x das posições inicial e final
    Y_inicioEfim = [];    % Vetor para as coordenadas y das posições inicial e final
    X_inicioEfim(1) = Xt1(1);
    Y_inicioEfim(1) = Xt1(2);
    x = []; y=[];
    x = Xt1(1);     % x é um vetor das coordenadas x de todas as posições preditas
    y = Xt1(2);     % y é um vetor das coordenadas y de todas as posições preditas
    tetas = [];
    for i = 1:500  % 1000 é a quantidade de amostra
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

