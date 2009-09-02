package br.com.r4j.math.kalmanfilter;

import JSci.maths.AbstractDoubleMatrix;


public interface DoubleVectorFunction
{
	/**
	 * Calcula a respota da fun��o para cada coluna de input, e coloca o resultado
	 * nas colunas da sa�da.
	 */
	public void calculate(AbstractDoubleMatrix input, AbstractDoubleMatrix output);
}

