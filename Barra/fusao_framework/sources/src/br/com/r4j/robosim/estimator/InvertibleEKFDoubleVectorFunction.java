package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleVector;


public interface InvertibleEKFDoubleVectorFunction extends EKFDoubleVectorFunction
{
	/**
	 * @param stateEstimatei_minus1 o estado na itera��o anterior para servir de referencial, caso necess�rio
	 * @returns o estado a a partir das leituras sensoriais.
	 */
	public AbstractDoubleVector produceInverseResults(AbstractDoubleVector stateEstimatei_minus1, AbstractDoubleVector vectReadings);

	public AbstractDoubleMatrix getInverseTransitionMatrixJacobian(AbstractDoubleVector readings);
}
