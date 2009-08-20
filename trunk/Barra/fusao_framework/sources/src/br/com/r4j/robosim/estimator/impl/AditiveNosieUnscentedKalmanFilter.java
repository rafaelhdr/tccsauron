package br.com.r4j.robosim.estimator.impl;

import java.util.ArrayList;
import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.Arrays;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.EstimationException;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;


public class AditiveNosieUnscentedKalmanFilter extends BaseAditiveUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieUnscentedKalmanFilter.class.getName());
	private static Log logUKF = LogFactory.getLog("ukf");
	private static Log logUKF_I = LogFactory.getLog("ukf_input");
	private static Log logUKF_covar = LogFactory.getLog("ukf_covar");
	private static Log logTime = LogFactory.getLog("time");

	
	public AditiveNosieUnscentedKalmanFilter()
	{
		super();
	}


	public String getName()
	{
		return "Filtro de Kalman Unscented";
	}

	
	protected void doEstimate()
	{
		logUKF.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
		logUKF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 7) + "\r\nstateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 10));

		this.setModifiers(stateEstimate.dimension());

		// predição
		AbstractDoubleVector vectDynReadings = new DoubleVector(dynModelSensor.getDataDimension());
		dynModelSensor.getData(vectDynReadings);
		AbstractDoubleSquareMatrix snsQ = dynModelSensor.getDataCovariance();
		logUKF.debug("snsQ: \r\n" + MatrixUtil.toString(snsQ, 9, 7));
		AbstractDoubleSquareMatrix Q = dynModel.getModelIncrementalCovariance(stateEstimate, vectDynReadings, dynModelSensor.getDataCovariance());
		logUKF.debug("vectDynReadings: \r\n" + MatrixUtil.toString(vectDynReadings, 9, 7) + "\r\nQ: \r\n" + MatrixUtil.toString(Q, 9, 7));

		// Calcula os sigmas iniciais
		double detQ = JSciMatrixMath.det(Q);
		logUKF.debug("detQ: " + detQ);
	
		// Verifica quais sensores possuem observacões.
		ArrayList listObsObjects = new ArrayList();  
		Iterator itSensModels = listSensModels.iterator();
		Iterator itSens = listSens.iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			Sensor sns = (Sensor) itSens.next();
			UKFDoubleVectorFunction snsFunc = (UKFDoubleVectorFunction) snsModel;

			boolean bResults = sns.hasNewData();
			if (bResults)
			{
				AbstractDoubleVector vectReadings = new DoubleVector(sns.getDataDimension());
				sns.getData(vectReadings);
				AbstractDoubleSquareMatrix snsR = sns.getDataCovariance();
				bResults = snsModel.canProduceObservations(vectReadings, snsR);
				if (bResults)
				{
					AbstractDoubleVector vectObservations = snsModel.getObservation(vectReadings);
					AbstractDoubleSquareMatrix R = snsModel.getObservationCovariance(snsR);
					logUKF.debug("R: \r\n" + MatrixUtil.toString(R, 9, 4));
					
					listObsObjects.add(snsModel);
					listObsObjects.add(vectObservations);
					listObsObjects.add(R);
				}
			}
		}
/*
		double delta = 3 - stateEstimate.dimension();
		double gamma = Math.sqrt(stateEstimate.dimension() + delta);
		double w0state = delta/(delta + stateEstimate.dimension());
		double w0cov = w0state - 1;
		double wi = 1.0/(delta + stateEstimate.dimension())/2;
/*/
		double delta = 3 - stateEstimate.dimension();
		double gamma = Math.sqrt(stateEstimate.dimension());
		double w0state = 0;
		double w0cov = 0;
		double wi = 1.0/(2*stateEstimate.dimension());
//*/		
		logUKF.debug("w0state = " + w0state + ", w0cov = " + w0cov + ", wi = " + wi + ", gamma: " + gamma + ", delta = " + delta);

		AbstractDoubleMatrix stateLastSigma = this.generateSigmaPoints(stateEstimate, stateCovarEstimate, gamma);
		if (stateLastSigma != null)
			logUKF.debug("stateLastSigma: \r\n" + MatrixUtil.toString(stateLastSigma, 9, 7));
		else
		{
			logUKF.debug("stateLastSigma: null!");
			throw new EstimationException("UKF: stateLastSigma: null. Não pode estimar.");
		}

		// Calcula os sigmas pela tranformação de estado
		AbstractDoubleMatrix sigmaStatei_iminus1 = new DoubleMatrix(stateLastSigma.rows(), stateLastSigma.columns());
		stateFunction.produceResults(stateLastSigma, 
									vectDynReadings, 
									new DoubleMatrix(stateLastSigma.rows(), stateLastSigma.columns()), 
									sigmaStatei_iminus1, 
									this.getCovariance());
		logUKF.debug("sigmaStatei_iminus1: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1, 9, 7));

		// 2 - Calcula os vetores ...
		AbstractDoubleVector statei_iminus1 = this.calculateMean(sigmaStatei_iminus1, w0state, wi);
		logUKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 7));
		AbstractDoubleSquareMatrix Pi_iminus1 = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, w0cov, wi);
		Pi_iminus1 = Pi_iminus1.add(Q);
		logUKF.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 9, 4));
		double det = JSciMatrixMath.det(Pi_iminus1);
		logUKF.debug("Pi_iminus1.isSymmetric(): " + Pi_iminus1.isSymmetric() + ", Pi_iminus1.det(): " + det);
		this.setPredictedState(statei_iminus1, Pi_iminus1);

/*
	  	AbstractDoubleMatrix [] arrayQSqrts = JSciMatrixMath.choleskyDecompose(Q);
		logUKF.debug("arrayQSqrts[0]: \r\n" + MatrixUtil.toString(arrayQSqrts[0], 9, 4));
		AbstractDoubleMatrix sigmaStatei_iminus1_meanless = MatrixUtil.colSubtract(sigmaStatei_iminus1, statei_iminus1);
		logUKF.debug("sigmaStatei_iminus1_meanless: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1_meanless, 9, 4));
  		AbstractDoubleMatrix sigmaStatei_iminus1_multiplied = sigmaStatei_iminus1_meanless.scalarMultiply(Math.sqrt(wi));
		logUKF.debug("sigmaStatei_iminus1_multiplied: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1_multiplied, 9, 4));
		AbstractDoubleMatrix sigmaStatei_iminus1_extended = MatrixUtil.colsAppend(
														sigmaStatei_iminus1_multiplied, 
														arrayQSqrts[0]);
		for (int i = 0; i < sigmaStatei_iminus1_extended.rows(); i++)
			sigmaStatei_iminus1_extended.setElement(i, 0, 0);
		AbstractDoubleMatrix [] arrayQR = JSciMatrixMath.qrDecompose((AbstractDoubleMatrix) sigmaStatei_iminus1_extended.transpose());
		AbstractDoubleMatrix [] arraySqrts = JSciMatrixMath.choleskyDecompose(Pi_iminus1);

		logUKF.debug("sigmaStatei_iminus1_extended: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1_extended, 9, 4));
		logUKF.debug("arrayQR[0]: \r\n" + MatrixUtil.toString(arrayQR[0], 9, 4));
		logUKF.debug("arrayQR[1]: \r\n" + MatrixUtil.toString(arrayQR[1], 9, 4));
		logUKF.debug("arraySqrts[0]: \r\n" + MatrixUtil.toString(arraySqrts[0], 9, 4));
		logUKF.debug("arraySqrts[1]: \r\n" + MatrixUtil.toString(arraySqrts[1], 9, 4));
//*/														

		Iterator itObsObjects = listObsObjects.iterator();
		while (itObsObjects.hasNext())
		{
			SensorModel snsModel = (SensorModel) itObsObjects.next();
			logUKF.debug("snsModel.getName(): " + snsModel.getName());
			UKFDoubleVectorFunction snsFunc = (UKFDoubleVectorFunction) snsModel;
			AbstractDoubleVector vectObservations = (AbstractDoubleVector) itObsObjects.next();
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) itObsObjects.next();
			logUKF.debug("R: \r\n" + MatrixUtil.toString(R, 9, 7));
			
			// Calcula os sigmas pela tranformação de medição
			DoubleMatrix obsPredSigma = new DoubleMatrix(vectObservations.dimension(), sigmaStatei_iminus1.columns());
			snsFunc.produceResults(sigmaStatei_iminus1, 
									statei_iminus1, 
									new DoubleMatrix(vectObservations.dimension(), 
									sigmaStatei_iminus1.columns()), 
									obsPredSigma, 
									Pi_iminus1);
			logUKF.debug("obsPredSigma: \r\n" + MatrixUtil.toString(obsPredSigma, 9, 7));

			AbstractDoubleVector obsPred = this.calculateMean(obsPredSigma, w0state, wi);

			AbstractDoubleSquareMatrix pYY = this.calculateCov(obsPredSigma, obsPred, w0cov, wi);
			logUKF.debug("pYY: \r\n" + MatrixUtil.toString(pYY, 9, 4));
			pYY = pYY.add(R);
			logUKF.debug("pYY.add(R): \r\n" + MatrixUtil.toString(pYY, 9, 4));

			logUKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 4));
			AbstractDoubleMatrix pXY = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, obsPredSigma, obsPred, w0cov, wi);
			logUKF.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 9, 4));

			AbstractDoubleSquareMatrix pYYInv = pYY.inverse();
			AbstractDoubleMatrix gain = pXY.multiply(pYYInv);
			logUKF.debug("gain: \r\n" + MatrixUtil.toString(gain, 9, 4));

			logUKF.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 9, 4));
			logUKF.debug("obsPred: \r\n" + MatrixUtil.toString(obsPred, 9, 4));
			AbstractDoubleVector obsDiff = vectObservations.subtract(obsPred);
			AbstractDoubleVector stateGain =  gain.multiply(obsDiff);
			logUKF.debug("stateGain: \r\n" + MatrixUtil.toString(stateGain, 9, 4));
			AbstractDoubleVector statei_i = statei_iminus1.add(stateGain);
			logUKF.debug("statei_i corr: \r\n" + MatrixUtil.toString(statei_i, 9, 4));

			AbstractDoubleSquareMatrix pXXcorrSim = (AbstractDoubleSquareMatrix) gain.multiply(pYY).multiply((AbstractDoubleMatrix) gain.transpose());
			logUKF.debug("pXXcorrSim: \r\n" + MatrixUtil.toString(pXXcorrSim, 9, 7));

			AbstractDoubleSquareMatrix Pi_i = (AbstractDoubleSquareMatrix) Pi_iminus1.subtract(pXXcorrSim);
			double detPi_i = JSciMatrixMath.det(Pi_i);
			logUKF.debug("detPi_i: " + detPi_i);
			
			if (detPi_i < 0)
			{
				double detPi_iminus1 = JSciMatrixMath.det(Pi_iminus1);
				double detCorr = JSciMatrixMath.det(pXXcorrSim);
				logUKF.debug("detPi_iminus1: " + detPi_iminus1 + ", detCorr: " + detCorr);
//				double alpha = Math.sqrt((detPi_iminus1/detCorr/10) / pXXcorrSim.rows());
				double alpha = Math.pow((detPi_iminus1/detCorr/10), 1.0/pXXcorrSim.rows());

				pXXcorrSim = (AbstractDoubleSquareMatrix) pXXcorrSim.scalarMultiply(alpha);
				logUKF.debug("alpha: \r\n" + alpha);
				logUKF.debug("pXXcorrSim: \r\n" + MatrixUtil.toString(pXXcorrSim, 9, 7));
				Pi_i = (AbstractDoubleSquareMatrix) Pi_iminus1.subtract(pXXcorrSim);
				logUKF.debug("Pi_i: \r\n" + MatrixUtil.toString(Pi_i, 9, 16));

				detPi_i = JSciMatrixMath.det(Pi_i);
				logUKF.debug("detPi_i again: " + detPi_i);
				if (detPi_i > 0)
				{
					statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;
					this.setState(statei_i, Pi_i);
				}
				else
				{
					this.setState(statei_i, Pi_iminus1);
					statei_iminus1 = statei_i;
				}
			}
			else
			{
				this.setState(statei_i, Pi_i);
				statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;
			}
		}
	}


	public static ModelFilter getModelFilter()
	{
		return new UKFModelFilter();
	}
}


class UKFModelFilter implements ModelFilter
{
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return (dynModel instanceof UKFDoubleVectorFunction);
	}


	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return (sensModel instanceof UKFDoubleVectorFunction);
	}
}

