package br.com.r4j.robosim.model;

import java.io.IOException;
import java.awt.*;
import java.util.*;
import java.awt.image.*;

import org.apache.commons.logging.LogFactory;
import org.apache.commons.logging.Log;

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
import br.com.r4j.robosim.estimator.PosteriorProbabilityDensityFunction;
import br.com.r4j.robosim.*;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.ParticleCloudPredictorFunction;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


public class OdometryModel implements DynamicModel, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, ParticleCloudPredictorFunction, MapDependent
{
	private static Log log = LogFactory.getLog(OdometryModel.class.getName());

	private WorldMap map = null;

	private AbstractDoubleMatrix F = null;
	private double d = 0;
	private double dThetaTrans = 0;
	private double dTheta = 0;
	private double sin = 0;
	private double cos = 0;

	private Random rnd = null;


	public OdometryModel()
	{
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


	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}


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
	public void produceResults(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix stateCovar, AbstractDoubleVector statePredictedOut)
	{
		d = sensorReadings.getComponent(0);
		dThetaTrans = sensorReadings.getComponent(1);
		dTheta = sensorReadings.getComponent(2);

		sin = Math.sin(stateBefore.getComponent(2) + dThetaTrans);
		cos = Math.cos(stateBefore.getComponent(2) + dThetaTrans);

		statePredictedOut.setComponent(0, stateBefore.getComponent(0) + d*cos);
		statePredictedOut.setComponent(1, stateBefore.getComponent(1) + d*sin);
		statePredictedOut.setComponent(2, stateBefore.getComponent(2) + dTheta);
	}


	public AbstractDoubleSquareMatrix getModelIncrementalCovariance(AbstractDoubleVector stateBefore, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		if (stateBefore != null)
		{
			d = sensorReadings.getComponent(0);
			dThetaTrans = sensorReadings.getComponent(1);
			dTheta = sensorReadings.getComponent(2);

			sin = Math.sin(stateBefore.getComponent(2) + dThetaTrans);
			cos = Math.cos(stateBefore.getComponent(2) + dThetaTrans);
			log.debug("stateBefore.getComponent(2) + dThetaTrans: " + (stateBefore.getComponent(2) + dThetaTrans));
		}
		log.debug("d: " + d + ", sin: " + sin + ", cos: " + cos);
		log.debug("dThetaTrans: " + dThetaTrans + ", dTheta: " + dTheta);


		AbstractDoubleSquareMatrix HF = new DoubleSquareMatrix(3);
//*		
		HF.setElement(0, 0, cos); HF.setElement(0, 1, -d*sin); HF.setElement(0, 2, 0);
		HF.setElement(1, 0, sin); HF.setElement(1, 1, d*cos);  HF.setElement(1, 2,  0);
		HF.setElement(2, 0, 0);   HF.setElement(2, 1, 0);      HF.setElement(2, 2, 1);
/*/		
		HF.setElement(0, 0, cos); HF.setElement(0, 1, 0); HF.setElement(0, 2, 0);
		HF.setElement(1, 0, sin); HF.setElement(1, 1, 0); HF.setElement(1, 2, 0);
		HF.setElement(2, 0, 0); HF.setElement(2, 1, 1); HF.setElement(2, 2, 0);
//*/		
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


	public void produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector sensorReadings, AbstractDoubleMatrix sigmaError, AbstractDoubleMatrix sigmaOut, AbstractDoubleSquareMatrix stateCovar)
	{
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
//*			
			d = sensorReadings.getComponent(0) + sigmaError.getElement(0, idxInput);
			dThetaTrans = sensorReadings.getComponent(1) + sigmaError.getElement(1, idxInput);
			dTheta = sensorReadings.getComponent(2) + sigmaError.getElement(2, idxInput);
/*/			
			d = sensorReadings.getComponent(0);
			dThetaTrans = sensorReadings.getComponent(1);
			dTheta = sensorReadings.getComponent(2);
//*/
			sin = Math.sin(sigmaIn.getElement(2, idxInput) + dThetaTrans);
			cos = Math.cos(sigmaIn.getElement(2, idxInput) + dThetaTrans);
			sigmaOut.setElement(0, idxInput, sigmaIn.getElement(0, idxInput) + d*cos);
			sigmaOut.setElement(1, idxInput, sigmaIn.getElement(1, idxInput) + d*sin);
			sigmaOut.setElement(2, idxInput, sigmaIn.getElement(2, idxInput) + dTheta);
		}
	}


	public void calculateNextStateCloud(AbstractDoubleMatrix input, AbstractDoubleMatrix output, AbstractDoubleVector sensorReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		d = sensorReadings.getComponent(0);
		dThetaTrans = sensorReadings.getComponent(1);
		dTheta = sensorReadings.getComponent(2);
		log.debug("calculateNextStateCloud:(d, dThetaTrans, dTheta): " + "(" + d + "," + dThetaTrans + "," + dTheta + ")");
		
		double detQ = JSciMatrixMath.det(sensorCovariance);
		log.debug("detQ: " + detQ);
		
		if (detQ == 0 || Double.isNaN(detQ))
		{
			double dThetaSin = Math.sin(dTheta), dThetaCos = Math.cos(dTheta);
			for (int idxInput = 0; idxInput < input.rows(); idxInput++)
			{
				sin = Math.sin(input.getElement(idxInput, 2) + dThetaTrans + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(1, 1)));
				cos = Math.cos(input.getElement(idxInput, 2) + dThetaTrans + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(1, 1)));
				double xRobot = input.getElement(idxInput, 0) + d*cos + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(0, 0));
				double yRobot = input.getElement(idxInput, 1) + d*sin + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(0, 0));
				double tRobot = input.getElement(idxInput, 2) + dTheta + rnd.nextGaussian()*Math.sqrt(sensorCovariance.getElement(2, 2));

				output.setElement(idxInput, 0, xRobot);
				output.setElement(idxInput, 1, yRobot);
				output.setElement(idxInput, 2, tRobot);
			}
		}
		else
		{		
			AbstractDoubleVector vectWhiteNoise = new DoubleVector(3);
			log.debug("calculateNextStateCloud:sensorCovariance: \r\n" + MatrixUtil.toString(sensorCovariance, 9, 4));
			AbstractDoubleMatrix [] arraySqrt = JSciMatrixMath.choleskyDecompose(sensorCovariance); 
			log.debug("calculateNextStateCloud:arraySqrt[0]: \r\n" + MatrixUtil.toString(arraySqrt[0], 9, 4));
			log.debug("calculateNextStateCloud:arraySqrt[1]: \r\n" + MatrixUtil.toString(arraySqrt[1], 9, 4));

			double dThetaSin = Math.sin(dTheta), dThetaCos = Math.cos(dTheta);
			for (int idxInput = 0; idxInput < input.rows(); idxInput++)
			{
				JSciMatrixMath.generateUnitaryWhiteNoise(vectWhiteNoise);
				AbstractDoubleVector vectNoise = arraySqrt[0].multiply(vectWhiteNoise);  
				sin = Math.sin(input.getElement(idxInput, 2) + dThetaTrans + vectNoise.getComponent(1));
				cos = Math.cos(input.getElement(idxInput, 2) + dThetaTrans + vectNoise.getComponent(1));
				double xRobot = input.getElement(idxInput, 0) + d*cos + vectNoise.getComponent(0);
				double yRobot = input.getElement(idxInput, 1) + d*sin + vectNoise.getComponent(0);
				double tRobot = input.getElement(idxInput, 2) + dTheta + vectNoise.getComponent(2);
				output.setElement(idxInput, 0, xRobot);
				output.setElement(idxInput, 1, yRobot);
				output.setElement(idxInput, 2, tRobot);
			}
			log.debug("input: \r\n" + MatrixUtil.toString((AbstractDoubleMatrix) input.transpose(), 9, 4));
			log.debug("output: \r\n" + MatrixUtil.toString((AbstractDoubleMatrix) output.transpose(), 9, 4));
		}
	}
}
