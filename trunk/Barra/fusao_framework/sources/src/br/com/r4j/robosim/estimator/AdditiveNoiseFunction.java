package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;


/** @modelguid {D962FBB4-2349-4FA3-B30F-35E28B8E051B} */
public interface AdditiveNoiseFunction
{
	/** @modelguid {4ECCA398-034C-45A1-A6F6-76723E36CFFD} */
	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar);
}
