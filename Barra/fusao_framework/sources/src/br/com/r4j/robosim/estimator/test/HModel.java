package br.com.r4j.robosim.estimator.test;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;



public class HModel implements SensorModel, Configurable, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction
{
	private static Log log = LogFactory.getLog(HModel.class.getName());

	private AbstractDoubleMatrix H = null;


	public HModel()
	{
		H = new DoubleMatrix(1, 2);
		H.setElement(0, 0, 1);
		H.setElement(0, 1, 0);
	}


	public void setSensor(Sensor sns)
	{
	}


	public String getName()
	{
		return "HModel";
	}

	
	public void configure(PropertiesHolder props, String strBaseKey)
	{
	}


	public void dataAvailable()
	{
	}


	public int getDataDimension()
	{
		return 1;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface DoubleVectorFunction

	/**
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector state, AbstractDoubleVector snsReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleVector obsPredicted = new DoubleVector(1);
		obsPredicted.setComponent(0, state.getComponent(0));
		return obsPredicted;
	}


	public AbstractDoubleSquareMatrix getObservationCovariance(AbstractDoubleSquareMatrix sensorCovariance)
	{
		return MatrixUtil.clone(sensorCovariance);
	}


	public AbstractDoubleVector getObservation(AbstractDoubleVector sensorReadings)
	{
		return sensorReadings;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface EKFDoubleVectorFunction

	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
		return H;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction

	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(sigmaIn.rows(), sigmaIn.columns()); 
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
			sigmaOut.setElement(0, idxInput, sigmaIn.getElement(0, idxInput));
		return sigmaOut;
	}

	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		return true;
	}


	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
	}
}

