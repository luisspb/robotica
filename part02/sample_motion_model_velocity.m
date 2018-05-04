function position = sample_motion_model_velocity(Ut,Xt1)
	
	v = Ut(1);
	w = Ut(2);
	x = Xt1(1);
	y = Xt1(2);
	teta = (Xt1(3)/180)*pi; % Passando pra radianos
    
    %alfa 1 to 6 são paramentros de erro de movimento especifico do robo.
    alfa1 = 0.2; alfa2 = 0.2; alfa3 = 0.2; alfa4 = 0.2; alfa5 = 0.2; alfa6 = 0.2;
    %DeltaT duração fixa de tempo.
    DeltaT = 3.0;
	%Não tenho absoluta cereza desses valores acima
	
	vc = v + sample(alfa1*abs(v) + alfa2*abs(w));
	wc = w + sample(alfa3*abs(v) + alfa4*abs(w));
	gamac = sample(alfa5*abs(v) + alfa6*abs(w));
	
	xl = x - (vc/wc)*sin(teta) + (vc/wc)*sin(teta + wc*DeltaT);
	yl = y + (vc/wc)*cos(teta) - (vc/wc)*cos(teta + wc*DeltaT);    
	tetaL = teta + wc*DeltaT + gamac*DeltaT;
	
	xt = [xl,yl,tetaL];	
	position = xt;
	
end
