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


public class AditiveNosieAugmentedUnscentedKalmanFilter extends BaseAditiveUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieAugmentedUnscentedKalmanFilter.class.getName());
	private static Log logUKF = LogFactory.getLog("aukf");
	private static Log logUKF_I = LogFactory.getLog("ukf_input");
	private static Log logUKF_covar = LogFactory.getLog("ukf_covar");
	private static Log logTime = LogFactory.getLog("time");

	
	public AditiveNosieAugmentedUnscentedKalmanFilter()
	{
		super();
	}


	public String getName()
	{
		return "Filtro de Kalman Unscented Aumentado";
	}

	
	protected void doEstimate()
	{
		logUKF.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
		logUKF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 7));
		logUKF.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 10));

		// predição
		AbstractDoubleVector vectDynReadings = new DoubleVector(dynModelSensor.getDataDimension());
		dynModelSensor.getData(vectDynReadings);
		AbstractDoubleSquareMatrix snsQ = dynModelSensor.getDataCovariance();
		logUKF.debug("snsQ: \r\n" + MatrixUtil.toString(snsQ, 9, 7));
		AbstractDoubleSquareMatrix Q = dynModel.getModelIncrementalCovariance(stateEstimate, vectDynReadings, snsQ);
		logUKF.debug("vectDynReadings: \r\n" + MatrixUtil.toString(vectDynReadings, 9, 7));
		logUKF.debug("Q: \r\n" + MatrixUtil.toString(Q, 9, 7));

		int totalSize = stateEstimate.dimension();

		// Calcula os sigmas iniciais
/*		
		double detQ = 1; for (int idxVar = 0; idxVar < Q.rows(); idxVar++)
			detQ *= Q.getElement(idxVar, idxVar);
/*/
		double detQ = snsQ.det();			
		logUKF.debug("detQ: " + detQ);

		// Indica que pode ser usado a matriz Q para aumentar o estado.
		if (detQ > 0)
			totalSize += Q.rows(); 

		
		// Verifica quais sensores possuem observacões.
		ArrayList listObsObjects = new ArrayList();  
		ArrayList listObsSigmaPoints = new ArrayList();  
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
					totalSize += R.rows();
				}
			}
		}

		this.setModifiers(totalSize);

		AbstractDoubleSquareMatrix stateCovarAugmented = new DoubleSquareMatrix(totalSize);
		AbstractDoubleVector stateAugmented = new DoubleVector(totalSize);
		int sizeCount = 0;
		MatrixUtil.copyBlock(sizeCount, stateCovarAugmented, stateCovarEstimate);
		MatrixUtil.copy(sizeCount, stateAugmented, stateEstimate);
		sizeCount += stateCovarEstimate.rows();
		if (detQ > 0)
		{
			MatrixUtil.copyBlock(sizeCount, stateCovarAugmented, snsQ);
			sizeCount += Q.rows();
		}
		for (int i = 0; i < listObsObjects.size(); i += 3)
		{
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) listObsObjects.get(i + 2);  
			MatrixUtil.copyBlock(sizeCount, stateCovarAugmented, R);
			sizeCount += R.rows();
		}
/*
		double delta = 3 - stateEstimate.dimension();
		double gamma = Math.sqrt(stateEstimate.dimension() + delta);
		double w0state = delta/(delta + stateEstimate.dimension());
		double w0cov = w0state - 1;
		double wi = 1.0/(delta + stateEstimate.dimension())/2;
/*/
		double delta = 3 - stateAugmented.dimension();
		double gamma = Math.sqrt(stateAugmented.dimension());
		double w0state = 0;
		double w0cov = 0;
		double wi = 1.0/(2.0*stateAugmented.dimension());
//*/		
		logUKF.debug("w0state = " + w0state + ", w0cov = " + w0cov + ", wi = " + wi + ", gamma: " + gamma + ", delta = " + delta);

		logUKF.debug("stateAugmented: \r\n" + MatrixUtil.toString(stateAugmented, 9, 7));
		logUKF.debug("stateCovarAugmented: \r\n" + MatrixUtil.toString(stateCovarAugmented, 9, 7));
		logUKF.debug("stateCovarAugmented.det(): " + stateCovarAugmented.det());
		AbstractDoubleMatrix stateLastSigmaAugmented = this.generateSigmaPoints(stateAugmented, stateCovarAugmented, gamma);

		if (stateLastSigmaAugmented != null)
			logUKF.debug("stateLastSigmaAugmented: \r\n" + MatrixUtil.toString(stateLastSigmaAugmented, 9, 7));
		else
		{
			logUKF.debug("stateLastSigmaAugmented: null!");
			throw new EstimationException("UKF: stateLastSigmaAugmented: null. Não pode estimar.");
		}

		// Pega os sigmas para cada função.
		AbstractDoubleMatrix stateLastSigma = MatrixUtil.getRows(stateLastSigmaAugmented, 0, stateEstimate.dimension());
		logUKF.debug("stateLastSigma: \r\n" + MatrixUtil.toString(stateLastSigma, 9, 7));
		sizeCount = stateEstimate.dimension();

		AbstractDoubleMatrix stateErrorSigma = null;
		if (detQ > 0)
		{
			stateErrorSigma = MatrixUtil.getRows(stateLastSigmaAugmented, sizeCount, Q.rows());
			sizeCount = Q.rows();
		}
		else
			stateErrorSigma = new DoubleMatrix(Q.rows(), stateLastSigmaAugmented.columns());
		logUKF.debug("stateErrorSigma: \r\n" + MatrixUtil.toString(stateErrorSigma, 9, 7));

		for (int i = 0; i < listObsObjects.size(); i += 3)
		{
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) listObsObjects.get(i + 2);  
			listObsSigmaPoints.add(MatrixUtil.getRows(stateLastSigmaAugmented, sizeCount, R.rows()));
			sizeCount += R.rows();
		}

		// Calcula os sigmas pela tranformação de estado
		AbstractDoubleMatrix sigmaStatei_iminus1 = new DoubleMatrix(stateLastSigma.rows(), stateLastSigma.columns());
		stateFunction.produceResults(stateLastSigma, vectDynReadings, stateErrorSigma, sigmaStatei_iminus1, this.getCovariance());
		logUKF.debug("sigmaStatei_iminus1: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1, 9, 7));

		// 2 - Calcula os vetores ...
		AbstractDoubleVector statei_iminus1 = this.calculateMean(sigmaStatei_iminus1, w0state, wi);
		logUKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 7));
		AbstractDoubleSquareMatrix Pi_iminus1 = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, w0cov, wi);
//		Pi_iminus1 = Pi_iminus1.add(Q);
		logUKF.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 9, 4));
		double det = Pi_iminus1.det();
		logUKF.debug("Pi_iminus1.isSymmetric(): " + Pi_iminus1.isSymmetric() + ", Pi_iminus1.det(): " + det);
		this.setPredictedState(statei_iminus1, Pi_iminus1);

		Iterator itObsObjects = listObsObjects.iterator(), itObsSigmaPoints = listObsSigmaPoints.iterator();
		while (itObsObjects.hasNext())
		{
			SensorModel snsModel = (SensorModel) itObsObjects.next();
			logUKF.debug("snsModel.getName(): " + snsModel.getName());
			UKFDoubleVectorFunction snsFunc = (UKFDoubleVectorFunction) snsModel;
			AbstractDoubleVector vectObservations = (AbstractDoubleVector) itObsObjects.next();
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) itObsObjects.next();
			AbstractDoubleMatrix sigmaErrorObs = (AbstractDoubleMatrix) itObsSigmaPoints.next();
			logUKF.debug("sigmaErrorObs: \r\n" + MatrixUtil.toString(sigmaErrorObs, 9, 7));
			
			// Calcula os sigmas pela tranformação de medição
			DoubleMatrix obsPredSigma = new DoubleMatrix(vectObservations.dimension(), sigmaStatei_iminus1.columns());
			snsFunc.produceResults(sigmaStatei_iminus1, statei_iminus1, sigmaErrorObs, obsPredSigma, Pi_iminus1);
			logUKF.debug("obsPredSigma: \r\n" + MatrixUtil.toString(obsPredSigma, 9, 7));

			AbstractDoubleVector obsPred = this.calculateMean(obsPredSigma, w0state, wi);
			logUKF.debug("obsPred: \r\n" + MatrixUtil.toString(obsPred, 9, 7));

			AbstractDoubleSquareMatrix pYY = this.calculateCov(obsPredSigma, obsPred, w0cov, wi);
			logUKF.debug("pYY: \r\n" + MatrixUtil.toString(pYY, 9, 4));
			pYY = pYY.add(R);

			AbstractDoubleMatrix pXY = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, obsPredSigma, obsPred, w0cov, wi);
			logUKF.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 9, 4));

			AbstractDoubleSquareMatrix pYYInv = pYY.inverse();
			AbstractDoubleMatrix gain = pXY.multiply(pYYInv);
			logUKF.debug("gain: \r\n" + MatrixUtil.toString(gain, 9, 4));

			logUKF.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 9, 7));

			AbstractDoubleVector obsDiff = vectObservations.subtract(obsPred);
			AbstractDoubleVector stateGain =  gain.multiply(obsDiff);
			AbstractDoubleVector statei_i = statei_iminus1.add(stateGain);

			AbstractDoubleMatrix gain_x_pYY_x_gainT = gain.multiply(pYY).multiply((AbstractDoubleMatrix) gain.transpose());
			AbstractDoubleSquareMatrix pXXcorrSim = MatrixUtil.convert2SquareMatrix(gain_x_pYY_x_gainT);
			logUKF.debug("pXXcorrSim: \r\n" + MatrixUtil.toString(pXXcorrSim, 9, 7));

			AbstractDoubleSquareMatrix Pi_i = MatrixUtil.convert2SquareMatrix(Pi_iminus1.subtract(pXXcorrSim));

			this.setState(statei_i, Pi_i);
			statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;
		}
	}


	public static ModelFilter getModelFilter()
	{
		return new AugUKFModelFilter();
	}
}


class AugUKFModelFilter implements ModelFilter
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

