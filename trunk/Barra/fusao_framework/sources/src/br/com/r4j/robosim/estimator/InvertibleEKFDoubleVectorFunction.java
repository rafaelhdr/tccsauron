package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleVector;


/** @modelguid {5CF376E6-8785-4725-A4C1-1DE96C0259E2} */
public interface InvertibleEKFDoubleVectorFunction extends EKFDoubleVectorFunction
{
	/**
	 * @param stateEstimatei_minus1 o estado na iteração anterior para servir de referencial, caso necessário
	 * @returns o estado a a partir das leituras sensoriais.
	 * @modelguid {BB06B16B-916F-4E49-BB2A-AEBA067F9FF0}
	 */
	public AbstractDoubleVector produceInverseResults(AbstractDoubleVector stateEstimatei_minus1, AbstractDoubleVector vectReadings);

	/** @modelguid {FCF79C25-677C-41B9-9F66-05562CBEDDDE} */
	public AbstractDoubleMatrix getInverseTransitionMatrixJacobian(AbstractDoubleVector readings);
}
