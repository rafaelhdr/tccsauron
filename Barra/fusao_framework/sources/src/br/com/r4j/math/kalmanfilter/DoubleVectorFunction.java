package br.com.r4j.math.kalmanfilter;

import JSci.maths.AbstractDoubleMatrix;


public interface DoubleVectorFunction
{
	/**
	 * Calcula a respota da função para cada coluna de input, e coloca o resultado
	 * nas colunas da saída.
	 */
	public void calculate(AbstractDoubleMatrix input, AbstractDoubleMatrix output);
}

