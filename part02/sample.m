function sampleRet = sample(b) 
	%b = variancia
	soma = 0;
	for indice = 1:12
		soma = soma + (-1 + 2 * rand(1)); %(-1 + 2 * rand(1) gera numeros aleatorios entre -1 e 1.
	end
	sampleRet = (b/6) * soma;
end