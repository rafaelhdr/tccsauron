package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


/** @modelguid {BDC3234D-E8FD-498F-96B2-8AD5AF97C143} */
public interface PosteriorProbabilityDensityFunction
{
	/** @modelguid {89ECBBA1-A1E9-42E0-B487-C986856FB74E} */
	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar);

	/** @modelguid {ECC5BBCB-04D7-4ECE-B8E0-F739CE1275CA} */
	public double stateProbability(AbstractDoubleMatrix input, int rowObjectCount);

	/** @modelguid {4DC0649B-6D57-4259-8F2B-738C4DA4FD41} */
	public void copyParticleData(int j, int i);

	/** @modelguid {8FE5FE49-FDC5-4BC9-9C76-732931262991} */
	public void setObservations(AbstractDoubleVector readings);

	/** @modelguid {47130239-00C6-440C-8497-D96E10517251} */
	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate);
}
