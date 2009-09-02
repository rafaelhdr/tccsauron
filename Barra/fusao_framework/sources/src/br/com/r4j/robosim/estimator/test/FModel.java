
package br.com.r4j.robosim.estimator.test;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


public class FModel implements DynamicModel, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction
{
	private static Log log = LogFactory.getLog(FModel.class.getName());

	private AbstractDoubleMatrix F = null;


	public FModel()
	{
		F = new DoubleMatrix(2, 2);

//  model.A  = [model.params(1) model.params(2); 1 0];

		F.setElement(0, 0, 1.9223);
		F.setElement(0, 1, -0.9604);

		F.setElement(1, 0, 1);
		F.setElement(1, 1, 0);
	}


	public String getName()
	{
		return "FModel";
	}

	
	public void setSensor(Sensor sns)
	{
	}


	public void dataAvailable()
	{
	}


	public int getDataDimension()
	{
		return 2;
	}


	public AbstractDoubleVector produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		/// inutil ... precisa sair ...
		AbstractDoubleVector statePredictedOut = new DoubleVector(1); 
		MatrixUtil.copy(0, statePredictedOut, F.multiply(stateBefore));
		return statePredictedOut;
	}


	public AbstractDoubleSquareMatrix getModelIncrementalCovariance(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		return MatrixUtil.clone(sensorCovariance);
	}


	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
		return F;
	}

	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(sigmaIn.rows(), sigmaIn.columns()); 
		MatrixUtil.copy(sigmaOut, F.multiply(sigmaIn));
		return sigmaOut;
	}


	public boolean canProduceObservations(AbstractDoubleVector readings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector state, AbstractDoubleSquareMatrix stateCovar)
	{
		return true;
	}


	public void correctByDimension(AbstractDoubleVector stateMean, AbstractDoubleSquareMatrix stateCovar,
													AbstractDoubleVector stateMeanNew, AbstractDoubleSquareMatrix stateCovarNew)
	{
	}
}
