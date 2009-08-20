package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;


public interface AdditiveNoiseFunction
{
	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar);
}
