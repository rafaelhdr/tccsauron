package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.*;


public interface PosteriorProbabilityDensityFunction
{
	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar);

	public double stateProbability(AbstractDoubleMatrix input, int rowObjectCount);

	public void copyParticleData(int j, int i);

	public void setObservations(AbstractDoubleVector readings);
}
