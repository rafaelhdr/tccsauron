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
import JSci.maths.DoubleVector;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.robosim.estimator.DoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.InvertibleEKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.PosteriorProbabilityDensityFunction;
import br.com.r4j.robosim.estimator.*;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.*;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.robosim.simrobot.SimRadarSensor;


public class RadarModel implements SensorModel, Configurable, DoubleVectorFunction, UKFDoubleVectorFunction, EKFDoubleVectorFunction, 	PosteriorProbabilityDensityFunction, InvertibleEKFDoubleVectorFunction, MapDependent
{
	private static Log log = LogFactory.getLog(RadarModel.class.getName());
	private static Log logSens = LogFactory.getLog("radar");

	private SimRadarSensor sns = null;
	private WorldMap map = null;

	private Random rnd = null;


	public RadarModel()
	{
		rnd = new Random((new Date()).getTime());
	}


	public void setSensor(Sensor sns)
	{
		this.sns = (SimRadarSensor) sns;
	}


	public void setWorldMap(WorldMap map)
	{
		this.map = map;
	}


	public String getName()
	{
		return "Radar Model";
	}

	
	public void configure(PropertiesHolder props, String strBaseKey)
	{
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


	public boolean canProduceObservations(AbstractDoubleVector vectReadings, AbstractDoubleSquareMatrix sensorCovariance)
	{
		return true;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface DoubleVectorFunction

	/**
	 * retorna true se é possível obter algum dado das leituras sensoriais.
	 */
	public void produceResults(AbstractDoubleVector state, AbstractDoubleVector obsPredicted, AbstractDoubleSquareMatrix stateCovar)
	{
		this.produceResults(state, obsPredicted);
	}


	protected void produceResults(AbstractDoubleVector state, AbstractDoubleVector obsPredicted)
	{
		double dX = state.getComponent(0) - sns.getRadarPose().getX();
		double dY = state.getComponent(1) - sns.getRadarPose().getY();

		if (dX > 0)
			obsPredicted.setComponent(0, Math.sqrt(dX*dX + dY*dY));
		else
			obsPredicted.setComponent(0, -Math.sqrt(dX*dX + dY*dY));

		if (dX*dX > 0.0001)
			if (dX > 0)
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(Math.atan(dY/dX), sns.getLastRads()));
			else
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(Math.atan(dY/dX) + Math.PI, sns.getLastRads()));
		else
			if (dY > 0)
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(Math.PI/2, sns.getLastRads()));
			else
				obsPredicted.setComponent(1, FunctionsR.aproxRadian(3*Math.PI/2, sns.getLastRads()));

		obsPredicted.setComponent(2, state.getComponent(2));
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
		AbstractDoubleMatrix H = new DoubleMatrix(3, 3);
		double dX = state.getComponent(0) - sns.getRadarPose().getX();
		double dY = state.getComponent(1) - sns.getRadarPose().getY();

		double e11 = 1, e12 = 1, e21 = 1, e22 = 1;

		double base2 = 1 + (dY*dY)/(dX*dX);
		e22 = 1/(dX*base2);
		e21 = -(dY/dX)*e22;

		double base1 = Math.sqrt(dX*dX + dY*dY);
		e11 = (dX/Math.abs(dX))*dX/base1;
		e12 = (dX/Math.abs(dX))*dY/base1;

		H.setElement(0, 0, e11); H.setElement(0, 1, e12); H.setElement(0, 2, 0);
		H.setElement(1, 0, e21); H.setElement(1, 1, e22); H.setElement(1, 2, 0);
		H.setElement(2, 0, 0); H.setElement(2, 1, 0); H.setElement(2, 2, 1);

		return H;
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface UKFDoubleVectorFunction

	public void produceResults(AbstractDoubleMatrix sigmaIn, AbstractDoubleVector state, AbstractDoubleMatrix sigmaError, AbstractDoubleMatrix sigmaOut, AbstractDoubleSquareMatrix stateCovar)
	{
		for (int idxInput = 0; idxInput < sigmaIn.columns(); idxInput++)
		{
			double dX = sigmaIn.getElement(0, idxInput) - sns.getRadarPose().getX();
			double dY = sigmaIn.getElement(1, idxInput) - sns.getRadarPose().getY();
			
			if (dX > 0)
				sigmaOut.setElement(0, idxInput, Math.sqrt(dX*dX + dY*dY) + sigmaError.getElement(0, idxInput));
			else
				sigmaOut.setElement(0, idxInput, -Math.sqrt(dX*dX + dY*dY) + sigmaError.getElement(0, idxInput));
				
			if (dX*dX > 0.0001)
				if (dX > 0)
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(Math.atan(dY/dX) + sigmaError.getElement(1, idxInput), sns.getLastRads()));
				else
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(Math.atan(dY/dX) + Math.PI + sigmaError.getElement(1, idxInput), sns.getLastRads()));
			else
				if (dY > 0)
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(Math.PI/2 + sigmaError.getElement(1, idxInput), sns.getLastRads()));
				else
					sigmaOut.setElement(1, idxInput, FunctionsR.aproxRadian(3*Math.PI/2 + sigmaError.getElement(1, idxInput), sns.getLastRads()));

			sigmaOut.setElement(2, idxInput, sigmaIn.getElement(2, idxInput) + sigmaError.getElement(2, idxInput));
		}
	}

//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface InvertibleEKFDoubleVectorFunction

	public AbstractDoubleVector produceInverseResults(AbstractDoubleVector stateEstimatei_minus1, AbstractDoubleVector readings)
	{
		AbstractDoubleVector output = new DoubleVector(3);

		double r = readings.getComponent(0);
		double theta = readings.getComponent(1);

		if (r < 0)
		{
			theta += Math.PI;
			r = -r;
		}

		output.setComponent(0, r*Math.cos(theta) + sns.getRadarPose().getX());
		output.setComponent(1, r*Math.sin(theta) + sns.getRadarPose().getY());
		output.setComponent(2, readings.getComponent(2));
		
		return output;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.estimator.InvertibleEKFDoubleVectorFunction#getInverseTransitionMatrixJacobian(JSci.maths.AbstractDoubleVector)
	 */
	public AbstractDoubleMatrix getInverseTransitionMatrixJacobian(AbstractDoubleVector readings)
	{
		AbstractDoubleMatrix H = new DoubleMatrix(3, 3);

		double r = readings.getComponent(0);
		double theta = readings.getComponent(1);

		if (r < 0)
		{
			theta += Math.PI;
			r = -r;
		}

		double e11 = Math.cos(theta);
		double e21 = Math.sin(theta);

		double e12 = -1*r*Math.sin(theta);
		double e22 = r*Math.cos(theta);
		H.setElement(0, 0, e11); H.setElement(0, 1, e12); H.setElement(0, 2, 0);
		H.setElement(1, 0, e21); H.setElement(1, 1, e22); H.setElement(1, 2, 0);
		H.setElement(2, 0, 0); H.setElement(2, 1, 0); H.setElement(2, 2, 1);
		return H;
	}


//	////////////////////////////////////////////////////////////////////////////////
//	////////////////////////////////////////////////////////////////////////////////
//	/// Interface PosteriorProbabilityDensityFunction

	private AbstractDoubleSquareMatrix covar = null;
	private AbstractDoubleSquareMatrix covarInv = null;
	private AbstractDoubleVector readings = null;
	private double pdfConst = 0;

	public void setAdditiveNoise(AbstractDoubleSquareMatrix covar)
	{
		this.covar = covar;
		covarInv = covar.inverse();
		log.debug("covar: \r\n" + MatrixUtil.toString(covar, 9, 4));
		log.debug("covarInv: \r\n" + MatrixUtil.toString(covarInv, 9, 4));
		pdfConst = 1.0/Math.sqrt(((AbstractDoubleSquareMatrix) covar.scalarMultiply(2*Math.PI)).det());
		log.debug("pdfConst: " + pdfConst);
		if (pdfConst < 1)
			pdfConst = 1;
		log.debug("pdfConst(2): " + pdfConst);
	}


	public void copyParticleData(int j, int i)
	{
	}


	public void setObservations(AbstractDoubleVector readings)
	{
		this.readings = readings;
	}


	public double stateProbability(AbstractDoubleMatrix input, int rowObjectCount)
	{
		DoubleVector vectVal = new DoubleVector(readings.dimension());
		DoubleVector inputVectVal = new DoubleVector(input.columns());
		for (int i = 0; i < input.columns(); i++)
			inputVectVal.setComponent(i, input.getElement(rowObjectCount, i));

		this.produceResults(inputVectVal, vectVal);
//		log.debug("inputVectVal: \r\n" + MatrixUtil.toString(inputVectVal, 9, 4));
//		log.debug("vectVal: \r\n" + MatrixUtil.toString(vectVal, 9, 4));
//		log.debug("readings: \r\n" + MatrixUtil.toString(readings, 9, 4));
		AbstractDoubleVector diff = vectVal.subtract(readings);
//		log.debug("diff: \r\n" + MatrixUtil.toString(diff, 9, 4));
		double expoent = covarInv.multiply(diff).scalarProduct(diff);
//		logPF.debug("covarInv: \r\n" + MatrixUtil.toString(covarInv, 9, 4));
//		logPF.debug("covarInv.multiply(diff): \r\n" + MatrixUtil.toString(covarInv.multiply(diff), 9, 4));
//		log.debug("expoent: " + expoent);
//		return pdfConst*Math.exp(-1.0/2.0*expoent + 20);
		return pdfConst*Math.exp(-1.0/2.0*expoent);
	}
}

