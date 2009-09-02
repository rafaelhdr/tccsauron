package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


/** @modelguid {C858CAD9-E0D5-4166-9EAD-EED1F6B20469} */
public interface ParticleCloudPredictorFunction
{
	/** @modelguid {0B5EBA60-BA24-42AB-A964-17EF6D7FCFD0} */
	public boolean canProduceObservationsPF(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate);
	
	public void calculateNextStateCloud(AbstractDoubleMatrix input, AbstractDoubleMatrix output, AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance);
}
