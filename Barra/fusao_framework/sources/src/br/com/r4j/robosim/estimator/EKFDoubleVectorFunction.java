package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


/** @modelguid {CE6C9265-50E0-43A6-AB15-85166A2AD0B8} */
public interface EKFDoubleVectorFunction extends DoubleVectorFunction
{
	/** @modelguid {23852DB0-A74F-4783-89C9-C5595791456A} */
	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state);

	public boolean canProduceObservations(AbstractDoubleVector readings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector state, AbstractDoubleSquareMatrix stateCovar);

	/**
	 * retorna true se é possível obter algum dado das leituras sensoriais.
	 * @modelguid {30C5B690-C37A-42FA-B8C8-FE86D606D10D}
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar);
}
