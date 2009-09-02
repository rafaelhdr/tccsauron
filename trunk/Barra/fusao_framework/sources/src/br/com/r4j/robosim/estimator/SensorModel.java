package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;


public interface SensorModel extends BaseModel, DataConsumer
{
	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance);


	public AbstractDoubleVector getObservation(AbstractDoubleVector sensorReadings);


	/**
	 * @param vectReadings
	 * @return
	 */
//	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance);


	/**
	 * Alerta para o SensorModel que um novo estado corrigido está disponível.
	 */
	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr);
}
