package br.com.r4j.robosim.estimator;

import JSci.maths.*;


public interface ParticleCloudPredictorFunction
{
	public void calculateNextStateCloud(AbstractDoubleMatrix input, AbstractDoubleMatrix output, AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance);
}
