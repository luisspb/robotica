function position = sample_motion_model_velocity(Ut,Xt1,tempo, alfa)
	
	v = Ut(1);
	w = Ut(2);
	x = Xt1(1);
	y = Xt1(2);
	teta = (Xt1(3)/180)*pi; % Passando pra radianos
    
    %alfa 1 to 6 são paramentros de erro de movimento especifico do robo.
    %alfa1 = 0.2; alfa2 = 0.2; alfa3 = 0.2; alfa4 = 0.2; alfa5 = 0.2; alfa6 = 0.2;
    %DeltaT duração fixa de tempo.
	DeltaT = tempo;
	
	vc = v + sample(alfa(1)*norm(v) + alfa(2)*norm(w));
	wc = w + sample(alfa(3)*norm(v) + alfa(4)*norm(w));
	gamac = sample(alfa(5)*abs(v) + alfa(6)*abs(w));
    
    f = (vc/wc);
	xl = x - f*sin(teta) + f*sin(teta + wc*DeltaT);
	yl = y + f*cos(teta) - f*cos(teta + wc*DeltaT);    
	tetaL = teta + wc*DeltaT + gamac*DeltaT;
	tetaL = (tetaL*180)/pi; % Convertendo para graus
    
	xt = [xl;yl;tetaL];	% acho que Ã© transposta (matriz coluna)
	position = xt;
	
end