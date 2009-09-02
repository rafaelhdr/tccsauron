package br.com.r4j.robosim.model;

import java.util.Date;
import java.util.Random;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.ParticleCloudPredictorFunction;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


public class OdometryModel implements DynamicModel, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, ParticleCloudPredictorFunction//, MapDependent
{
	private static Log log = LogFactory.getLog(OdometryModel.class.getName());

//	private WorldMap map = null;

	private AbstractDoubleMatrix F = null;
	private double d = 0;
	private double dTheta = 0;
	private double sin = 0;
	private double cos = 0;

	private Random rnd = null;


	public OdometryModel()
	{
		if (log.isDebugEnabled())
			log.debug("OdometryModel: nova instância");
		
		F = new DoubleMatrix(3, 3);

		F.setElement(0, 0, 1);
		F.setElement(0, 1, 0);

		F.setElement(1, 0, 0);
		F.setElement(1, 1, 1);

		F.setElement(2, 0, 0);
		F.setElement(2, 1, 0);
		F.setElement(2, 2, 1);

		rnd = new Random((new Date()).getTime());
	}


	public String getName()
	{
		return "Odometry Model";
	}

	
	public void setSensor(Sensor sns)
	{
	}

/*
	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}
//*/


	/** 
	 * Método invocado quando os dados estiverem disponíveis.
	 *
	 */
	public void dataAvailable()
	{
		// não precisa fazer nada ...
	}


	public int getDataDimension()
	{
		return 3;
	}

	/**
	 */
	public AbstractDoubleVector produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar)
	{
		d = sensorReadings.getComponent(0);
		dTheta = sensorReadings.getComponent(1);
		sin = Math.sin(stateBefore.getComponent(2)); cos = Math.cos(stateBefore.getComponent(2));

		AbstractDoubleVector statePredictedOut = new DoubleVector(3); 
		statePredictedOut.setComponent(0, stateBefore.getComponent(0) + d*cos);
		statePredictedOut.setComponent(1, stateBefore.getComponent(1) + d*sin);
		statePredictedOut.setComponent(2, stateBefore.getComponent(2) + dTheta);
		
		return statePredictedOut;
	}


	public AbstractDoubleSquareMatrix getModelIncrementalCovariance(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		if (stateBefore != null)
		{
			d = sensorReadings.getComponent(0);
			dTheta = sensorReadings.getComponent(1);
			sin = Math.sin(stateBefore.getComponent(2)); cos = Math.cos(stateBefore.getComponent(2));
		}

		AbstractDoubleMatrix HF = new DoubleMatrix(3, 2);
		HF.setElement(0, 0, cos); HF.setElement(0, 1, 0);
		HF.setElement(1, 0, sin); HF.setElement(1, 1, 0);
		HF.setElement(2, 0, 0);   HF.setElement(2, 1, 1);
		AbstractDoubleMatrix HFT = (AbstractDoubleMatrix) HF.transpose();
		return MatrixUtil.convert2SquareMatrix(HF.multiply(sensorCovariance).multiply(HFT));
	}


	public AbstractDoubleMatrix getTransitionMatrixJacobian(AbstractDoubleVector state)
	{
//		double d = Math.sqrt(dX*dX + dY*dY);
//		double dThetaMean = state.getElement() - dTheta/2;

		F.setElement(0, 2, -d*sin);
		F.setElement(1, 2, d*cos);

		return F;
	}


	public AbstractDoubleMatrix produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleSquareMatrix stateCovar)
	{
		AbstractDoubleMatrix sigmaOut = new DoubleMatrix(sigmaIn.rows(), sigmaIn.columns()); 
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
			d = sensorReadings.getComponent(0) + sigmaError.getElement(0, idxInput);
			dTheta = sensorReadings.getComponent(1) + sigmaError.getElement(1, idxInput);

			sin = Math.sin(sigmaIn.getElement(2, idxInput)); cos = Math.cos(sigmaIn.getElement(2, idxInput));
			sigmaOut.setElement(0, idxInput, sigmaIn.getElement(0, idxInput) + d*cos);
			sigmaOut.setElement(1, idxInput, sigmaIn.getElement(1, idxInput) + d*sin);
			sigmaOut.setElement(2, idxInput, sigmaIn.getElement(2, idxInput) + dTheta);
		}
		return sigmaOut;
	}


	public void calculateNextStateCloud(AbstractDoubleMatrix input, AbstractDoubleMatrix output, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		d = sensorReadings.getComponent(0);
		dTheta = sensorReadings.getComponent(1);
		
		double detQ = JSciMatrixMath.det(sensorCovariance);
		
		if (detQ == 0 || Double.isNaN(detQ))
		{
			double dThetaSin = Math.sin(dTheta), dThetaCos = Math.cos(dTheta);
			for (int idxInput = 0; idxInput < input.rows(); idxInput++)
			{
				sin = Math.sin(input.getElement(idxInput, 2)); cos = Math.cos(input.getElement(idxInput, 2));
				double xRobot = input.getElement(idxInput, 0) + d*cos + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(0, 0));
				double yRobot = input.getElement(idxInput, 1) + d*sin + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(0, 0));
				double tRobot = input.getElement(idxInput, 2) + dTheta + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(1, 1));
				output.setElement(idxInput, 0, xRobot);
				output.setElement(idxInput, 1, yRobot);
				output.setElement(idxInput, 2, tRobot);
			}
		}
		else
		{		
			AbstractDoubleVector vectWhiteNoise = new DoubleVector(2);
			AbstractDoubleMatrix [] arraySqrt = JSciMatrixMath.choleskyDecompose(sensorCovariance); 

			double dThetaSin = Math.sin(dTheta), dThetaCos = Math.cos(dTheta);
			for (int idxInput = 0; idxInput < input.rows(); idxInput++)
			{
				JSciMatrixMath.generateUnitaryWhiteNoise(vectWhiteNoise);
				AbstractDoubleVector vectNoise = arraySqrt[0].multiply(vectWhiteNoise);  

				sin = Math.sin(input.getElement(idxInput, 2)); cos = Math.cos(input.getElement(idxInput, 2));
				double xRobot = input.getElement(idxInput, 0) + d*cos + vectNoise.getComponent(0);
				double yRobot = input.getElement(idxInput, 1) + d*sin + vectNoise.getComponent(0);
				double tRobot = input.getElement(idxInput, 2) + dTheta + vectNoise.getComponent(1);
				output.setElement(idxInput, 0, xRobot);
				output.setElement(idxInput, 1, yRobot);
				output.setElement(idxInput, 2, tRobot);
			}
//			log.debug("input: \r\n" + MatrixUtil.toString((AbstractDoubleMatrix) input.transpose(), 9, 4));
//			log.debug("output: \r\n" + MatrixUtil.toString((AbstractDoubleMatrix) output.transpose(), 9, 4));
		}
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.UKFDoubleVectorFunction#canProduceObservations(JSci.maths.AbstractDoubleVector, JSci.maths.AbstractDoubleSquareMatrix, JSci.maths.AbstractDoubleVector, JSci.maths.AbstractDoubleSquareMatrix)
	 */
	public boolean canProduceObservations(AbstractDoubleVector readings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector state, AbstractDoubleSquareMatrix stateCovar)
	{
		return false;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.ParticleCloudPredictorFunction#canProduceObservationsPF(JSci.maths.AbstractDoubleVector, JSci.maths.AbstractDoubleSquareMatrix, JSci.maths.AbstractDoubleVector, JSci.maths.AbstractDoubleSquareMatrix)
	 */
	public boolean canProduceObservationsPF(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance, AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		return true;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.DynamicModel#correctCovarByDimension(JSci.maths.AbstractDoubleSquareMatrix)
	 */
	public void correctByDimension(AbstractDoubleVector stateMean, AbstractDoubleSquareMatrix stateCovar,
													AbstractDoubleVector stateMeanNew, AbstractDoubleSquareMatrix stateCovarNew)
	{
		// copia o movimento
		for (int r = 0; r < 3; r++)
				stateMeanNew.setComponent(r, stateMean.getComponent(r));
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				stateCovarNew.setElement(r, c, stateCovar.getElement(r, c));
	}


	public void correctedState(AbstractDoubleVector meanPred, AbstractDoubleSquareMatrix covarPred, AbstractDoubleVector meanCorr, AbstractDoubleSquareMatrix covarCorr)
	{
	}
}
