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
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EstimationException;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.research.image.sequence.estimator.*;


public class AditiveNosieUnscentedKalmanFilter extends BaseAditiveUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieUnscentedKalmanFilter.class.getName());
	private static Log logUKF = LogFactory.getLog("ukf");
	private static Log logTime = LogFactory.getLog("time");

	
	public AditiveNosieUnscentedKalmanFilter()
	{
		super();
		this.logEstimator = logUKF;
	}

	
	public String getName()
	{
//		return "Filtro de Kalman Unscented";
		return "UKF";
	}


	protected void doEstimate()
	{
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
			logEstimator.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 7, 3) + "\r\nstateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 7, 3));
		}

		// predição
		AbstractDoubleVector vectDynReadings = new DoubleVector(dynModelSensor.getDataDimension(dynModel));
		dynModelSensor.getData(vectDynReadings, dynModel);
		AbstractDoubleSquareMatrix snsQ = dynModelSensor.getDataCovariance(dynModel);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("snsQ: \r\n" + MatrixUtil.toString(snsQ, 7, 3));

		// Calcula os sigmas iniciais
		this.setModifiers(stateEstimate.dimension());
		AbstractDoubleMatrix stateLastSigma = this.generateSigmaPoints(stateEstimate, stateCovarEstimate);
		if (stateLastSigma == null)
		{
			logEstimator.warn("stateLastSigma: null!");
			throw new EstimationException("UKF: stateLastSigma: null. Não pode estimar.");
		}

		// Calcula os sigmas pela tranformação de estado
		AbstractDoubleMatrix sigmaStatei_iminus1 = stateFunction.produceResults(stateLastSigma,
														stateEstimate, 
														vectDynReadings, 
														new DoubleMatrix(stateLastSigma.rows(), stateLastSigma.columns()), 
														this.getCovariance());
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("sigmaStatei_iminus1: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1, 7, 3));
		this.addSigmaPointsPredicted(sigmaStatei_iminus1);

		// 2 - Calcula os vetores ...
		AbstractDoubleVector statei_iminus1 = this.calculateMean(sigmaStatei_iminus1);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 7, 3));
		AbstractDoubleSquareMatrix Pi_iminus1 = this.calculateCov(sigmaStatei_iminus1, statei_iminus1);
//		if (logEstimator.isDebugEnabled())
//			logEstimator.debug("Pi_iminus1(B): \r\n" + MatrixUtil.toString(Pi_iminus1, 7, 3));


		// Verifica quais sensores possuem observacões.
		ArrayList listObsObjects = new ArrayList();  
		Iterator itSensModels = listSensModels.iterator();
		Iterator itSens = listSens.iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			Sensor sns = (Sensor) itSens.next();
			UKFDoubleVectorFunction snsFunc = (UKFDoubleVectorFunction) snsModel;
		
			boolean bResults = sns.hasNewData(snsModel, this);
			if (bResults)
			{
				AbstractDoubleVector vectReadings = new DoubleVector(sns.getDataDimension(snsModel));
				sns.getData(vectReadings, snsModel);
				AbstractDoubleSquareMatrix snsR = sns.getDataCovariance(snsModel);
				bResults = snsFunc.canProduceObservations(vectReadings, snsR, statei_iminus1, Pi_iminus1);
				if (bResults)
				{
					AbstractDoubleVector vectObservations = snsModel.getObservation(vectReadings);
					AbstractDoubleSquareMatrix R = snsModel.getObservationCovariance(snsR);
					if (logEstimator.isDebugEnabled())
						logEstimator.debug("R: \r\n" + MatrixUtil.toString(R, 7, 3));
							
					listObsObjects.add(snsModel);
					listObsObjects.add(vectObservations);
					listObsObjects.add(R);
				}
			}
		}

		// Corrige os vetores para o novo estado.
		AbstractDoubleVector stateEstimateNew = new DoubleVector(dynModel.getDataDimension());
		AbstractDoubleSquareMatrix stateCovarEstimateNew = new DoubleSquareMatrix(dynModel.getDataDimension());
		dynModel.correctByDimension(statei_iminus1, Pi_iminus1, stateEstimateNew, stateCovarEstimateNew);
		statei_iminus1 = stateEstimateNew;
		Pi_iminus1 = stateCovarEstimateNew;
		this.setModifiers(dynModel.getDataDimension());


		AbstractDoubleSquareMatrix Q = dynModel.getModelIncrementalCovariance(stateEstimate, vectDynReadings, dynModelSensor.getDataCovariance(dynModel));
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("vectDynReadings: \r\n" + MatrixUtil.toString(vectDynReadings, 7, 3));
			logEstimator.debug("Q: \r\n" + MatrixUtil.toString(Q, 7, 3));
		}
		Pi_iminus1 = Pi_iminus1.add(Q);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 7, 3));
		this.setPredictedState(statei_iminus1, Pi_iminus1);



		Iterator itObsObjects = listObsObjects.iterator();
		if (!itObsObjects.hasNext())
			this.addSigmaPointsFiltered(this.generateSigmaPoints(statei_iminus1, Pi_iminus1));

		while (itObsObjects.hasNext())
		{
			SensorModel snsModel = (SensorModel) itObsObjects.next();
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("snsModel.getName(): " + snsModel.getName());
			if (snsModel instanceof VLinesStateSensorModel)
			{
				((VLinesStateSensorModel) snsModel).correctAngulation(statei_iminus1, Pi_iminus1);
				this.setPredictedState(statei_iminus1, Pi_iminus1);
//				logEstimator.debug("ang corrected: statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 7, 3));
				this.setVLinesStateModifiers(dynModel.getDataDimension());
			}
			else
				this.setModifiers(dynModel.getDataDimension());

			sigmaStatei_iminus1 = this.generateSigmaPoints(statei_iminus1, Pi_iminus1);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("sigmaStatei_iminus1: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1, 7, 3));
			this.addSigmaPointsPredicted(sigmaStatei_iminus1);

			UKFDoubleVectorFunction snsFunc = (UKFDoubleVectorFunction) snsModel;
			AbstractDoubleVector vectObservations = (AbstractDoubleVector) itObsObjects.next();
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) itObsObjects.next();
			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 7, 3));
				logEstimator.debug("R: \r\n" + MatrixUtil.toString(R, 7, 3));
			}
/* DBG: limpa os sigma points para testar uma variação por sigma point
			AbstractDoubleMatrix sigmaStatei_iminus1Tmp = MatrixUtil.clone(sigmaStatei_iminus1);
			for (int idxState = 0; idxState < sigmaStatei_iminus1.rows(); idxState++)
				for (int i = 1; i < sigmaStatei_iminus1.columns(); i++)
					if ((i - 1)%sigmaStatei_iminus1.rows() != idxState)
						sigmaStatei_iminus1Tmp.setElement(idxState, i, sigmaStatei_iminus1.getElement(idxState, 0));
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("sigmaStatei_iminus1Tmp: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1Tmp, 7, 3));
//*/

			// Calcula os sigmas pela tranformação de medição
//			AbstractDoubleMatrix obsPredSigma = snsFunc.produceResults(sigmaStatei_iminus1Tmp, 
			AbstractDoubleMatrix obsPredSigma = snsFunc.produceResults(sigmaStatei_iminus1, 
											statei_iminus1,
											vectObservations, 
											new DoubleMatrix(vectObservations.dimension(), 
											sigmaStatei_iminus1.columns()), 
											Pi_iminus1);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("obsPredSigma: \r\n" + MatrixUtil.toString(obsPredSigma, 7, 3));

			AbstractDoubleVector obsPred = this.calculateMean(obsPredSigma);

			AbstractDoubleSquareMatrix pYY = this.calculateCov(obsPredSigma, obsPred);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("pYY: \r\n" + MatrixUtil.toString(pYY, 7, 3));
			pYY = pYY.add(R);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("pYY.add(R): \r\n" + MatrixUtil.toString(pYY, 7, 3));

			if (logEstimator.isDebugEnabled())
				logEstimator.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 7, 3));
//			AbstractDoubleMatrix pXY = this.calculateCov(sigmaStatei_iminus1Tmp, statei_iminus1, obsPredSigma, obsPred);
			AbstractDoubleMatrix pXY = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, obsPredSigma, obsPred);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 7, 3));

			AbstractDoubleSquareMatrix pYYInv = pYY.inverse();
			AbstractDoubleMatrix gain = pXY.multiply(pYYInv);
			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("pYYInv: \r\n" + MatrixUtil.toString(pYYInv, 7, 3));
				logEstimator.debug("gain: \r\n" + MatrixUtil.toString(gain, 7, 3));
				logEstimator.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 7, 3));
				logEstimator.debug("obsPred: \r\n" + MatrixUtil.toString(obsPred, 7, 3));
			}
			AbstractDoubleVector obsDiff = vectObservations.subtract(obsPred);
			AbstractDoubleVector stateGain =  gain.multiply(obsDiff);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("stateGain: \r\n" + MatrixUtil.toString(stateGain, 7, 3));
			AbstractDoubleVector statei_i = statei_iminus1.add(stateGain);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("statei_i corr: \r\n" + MatrixUtil.toString(statei_i, 7, 3));

			if (logEstimator.isDebugEnabled())
			{
				AbstractDoubleVector obsPredAfter = ((EKFDoubleVectorFunction) snsFunc).produceResults(statei_i, vectObservations, Pi_iminus1);
				logEstimator.debug("obs pred corr: \r\n" + MatrixUtil.toString(obsPredAfter, 7, 3));
			}



			AbstractDoubleSquareMatrix pXXcorrSim = (AbstractDoubleSquareMatrix) gain.multiply(pYY).multiply((AbstractDoubleMatrix) gain.transpose());
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("pXXcorrSim: \r\n" + MatrixUtil.toString(pXXcorrSim, 7, 3));

			AbstractDoubleSquareMatrix Pi_i = (AbstractDoubleSquareMatrix) Pi_iminus1.subtract(pXXcorrSim);
			double detPi_i = JSciMatrixMath.det(Pi_i);
			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("detPi_i: " + detPi_i);
				logEstimator.debug("Pi_i: \r\n" + MatrixUtil.toString(Pi_i, 7, 3));
			}
			
			if (detPi_i < 0)
			{
				if (logEstimator.isDebugEnabled())
					logEstimator.debug("correção de erro falhô. Falhô! . . .  Falhô!Falhô!");

				if (!itObsObjects.hasNext())
					this.addSigmaPointsFiltered(this.generateSigmaPoints(statei_i, Pi_iminus1));
				this.setState(statei_i, Pi_iminus1);
				statei_iminus1 = statei_i; Pi_iminus1 = Pi_iminus1;
			}
			else
			{
				if (!itObsObjects.hasNext())
					this.addSigmaPointsFiltered(this.generateSigmaPoints(statei_i, Pi_i));
				this.setState(statei_i, Pi_i);
				statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;
			}

			if (snsModel instanceof VLinesStateSensorModel)
			{
				((VLinesStateSensorModel) snsModel).setLastState(statei_i);
			}
		}

		itSensModels = listSensModels.iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			snsModel.correctedState(this.getPredictedMean(), this.getPredictedCovariance(), this.getMean(), this.getCovariance());
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

