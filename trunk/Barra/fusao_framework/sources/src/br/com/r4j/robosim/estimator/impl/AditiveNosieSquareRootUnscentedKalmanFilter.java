package br.com.r4j.robosim.estimator.impl;

import java.util.ArrayList;
import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.UKFDoubleVectorFunction;
import br.com.r4j.research.image.sequence.estimator.*;


public class AditiveNosieSquareRootUnscentedKalmanFilter extends BaseAditiveUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieSquareRootUnscentedKalmanFilter.class.getName());
	private static Log logUKF = LogFactory.getLog("srukf");

	private AbstractDoubleMatrix S = null;


	public AditiveNosieSquareRootUnscentedKalmanFilter()
	{
		super();
		this.logEstimator = logUKF;
	}


	public String getName()
	{
		return "Filtro de Kalman Unscented Square Root";
	}


	public void setState(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		// Inicia.
		if (S == null)
			S = JSciMatrixMath.choleskyDecompose(stateCovarEstimate)[1];

		super.setState(stateEstimate, stateCovarEstimate);
	}


	protected void doEstimate()
	{
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
			logEstimator.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 7, 3));
			logEstimator.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 10));
		}

		// predição
		AbstractDoubleVector vectDynReadings = new DoubleVector(dynModelSensor.getDataDimension(dynModel));
		dynModelSensor.getData(vectDynReadings, dynModel);
		AbstractDoubleSquareMatrix snsQ = dynModelSensor.getDataCovariance(dynModel);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("snsQ: \r\n" + MatrixUtil.toString(snsQ, 7, 3));

		// Calcula os sigmas iniciais
		this.setModifiers(stateEstimate.dimension());
		AbstractDoubleMatrix stateLastSigma = this.generateSigmaPointsWithSqrtCovar(stateEstimate, S);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("stateLastSigma: \r\n" + MatrixUtil.toString(stateLastSigma, 7, 3));


		

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
		S = this.getSi_minus1(sigmaStatei_iminus1, statei_iminus1, MatrixUtil.zeroes(statei_iminus1.dimension()));
		AbstractDoubleSquareMatrix Pi_iminus1 = MatrixUtil.convert2SquareMatrix(((AbstractDoubleMatrix) S.transpose()).multiply(S));
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("S: \r\n" + MatrixUtil.toString(S, 7, 3));
			logEstimator.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 7, 3));
		}

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

		sigmaStatei_iminus1 = this.generateSigmaPoints(statei_iminus1, Pi_iminus1);
		AbstractDoubleSquareMatrix Q = dynModel.getModelIncrementalCovariance(stateEstimate, vectDynReadings, snsQ);
		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("vectDynReadings: \r\n" + MatrixUtil.toString(vectDynReadings, 7, 3));
			logEstimator.debug("Q: \r\n" + MatrixUtil.toString(Q, 7, 3));
		}
		Pi_iminus1 = Pi_iminus1.add(Q);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 7, 3));
		this.setPredictedState(statei_iminus1, Pi_iminus1);

		S = JSciMatrixMath.choleskyDecompose(Pi_iminus1)[1];
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("S(dididi): \r\n" + MatrixUtil.toString(S, 7, 3));

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
				logEstimator.debug("ang corrected: statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 7, 3));
				this.setVLinesStateModifiers(dynModel.getDataDimension());
			}
			else
				this.setModifiers(dynModel.getDataDimension());

			sigmaStatei_iminus1 = this.generateSigmaPoints(statei_iminus1, Pi_iminus1);
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
			AbstractDoubleMatrix Sy = this.getSi_minus1(obsPredSigma, obsPred, R);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("Sy: \r\n" + MatrixUtil.toString(Sy, 7, 3));

			AbstractDoubleSquareMatrix pYYTeste = this.calculateCov(obsPredSigma, obsPred);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("pYYTeste: \r\n" + MatrixUtil.toString(pYYTeste, 7, 3));

			// Precisa trocar pelo squares ...
			AbstractDoubleSquareMatrix Pyy = MatrixUtil.convert2SquareMatrix(((AbstractDoubleMatrix) Sy.transpose()).multiply(Sy));
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("Pyy: \r\n" + MatrixUtil.toString(Pyy, 7, 3));
			AbstractDoubleMatrix PyyInv = Pyy.inverse();
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("PyyInv: \r\n" + MatrixUtil.toString(PyyInv, 7, 3));
			
			AbstractDoubleMatrix pXY = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, obsPredSigma, obsPred);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 7, 3));
			
			AbstractDoubleMatrix gain = pXY.multiply(PyyInv);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("gain: \r\n" + MatrixUtil.toString(gain, 7, 3));
			
			AbstractDoubleMatrix U = gain.multiply((AbstractDoubleMatrix) Sy.transpose());
			
			// Teste ...
			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("U: \r\n" + MatrixUtil.toString(U, 7, 3));
				logEstimator.debug("S: \r\n" + MatrixUtil.toString(S, 7, 3));
				
				AbstractDoubleMatrix S2 = ((AbstractDoubleMatrix) S.transpose()).multiply(S);
				AbstractDoubleMatrix U2 = (U).multiply((AbstractDoubleMatrix) U.transpose());
				logEstimator.debug("S2: \r\n" + MatrixUtil.toString(S2, 7, 3));
				logEstimator.debug("U2: \r\n" + MatrixUtil.toString(U2, 7, 3));

				AbstractDoubleMatrix S2mU2 = S2.subtract(U2);
				logEstimator.debug("S2mU2: \r\n" + MatrixUtil.toString(S2mU2, 7, 3));
				logEstimator.debug("S(B): \r\n" + MatrixUtil.toString(S, 7, 3));
			}

			S = (AbstractDoubleMatrix) JSciMatrixMath.cholupdate(S, U, -1);			
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("S(A): \r\n" + MatrixUtil.toString(S, 7, 3));

			AbstractDoubleSquareMatrix Pi_i = MatrixUtil.convert2SquareMatrix(((AbstractDoubleMatrix) S.transpose()).multiply(S));
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("Pi_i: \r\n" + MatrixUtil.toString(Pi_i, 7, 3));

			AbstractDoubleVector obsDiff = vectObservations.subtract(obsPred);
			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 7, 3));
				logEstimator.debug("obsPred: \r\n" + MatrixUtil.toString(obsPred, 7, 3));
			}
			AbstractDoubleVector stateGain =  gain.multiply(obsDiff);
			AbstractDoubleVector statei_i = statei_iminus1.add(stateGain);

			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("stateGain : \r\n" + MatrixUtil.toString(stateGain, 7, 3));
				logEstimator.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 7, 3));
				logEstimator.debug("statei_i: \r\n" + MatrixUtil.toString(statei_i, 7, 3));
			}

			this.setState(statei_i, Pi_i);
			statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;

			if (!itObsObjects.hasNext())
				this.addSigmaPointsFiltered(this.generateSigmaPoints(statei_i, Pi_i));

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


	/**
	 * @param sigmaStatei_iminus1
	 * @param statei_iminus1
	 * @param Q
	 * @return uma matriz triangular superior
	 * @modelguid {D5704BEF-E47F-4A74-BB4E-A0CFF981B71C}
	 */
	private AbstractDoubleMatrix getSi_minus1(AbstractDoubleMatrix sigmaPoints, AbstractDoubleVector mean, AbstractDoubleSquareMatrix covError)
	{
		double detCovError = JSciMatrixMath.det(covError);
		AbstractDoubleMatrix covErrorSqrt = null;

		if (detCovError == 0 || Double.isNaN(detCovError)) // (Math.abs(detCovError) < 1E-16)
		{
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("detCovError = " + detCovError);
			covErrorSqrt = covError;
		}
		else
		{
			AbstractDoubleMatrix [] arrayErrorSqrts = JSciMatrixMath.choleskyDecompose(covError);
			if (logEstimator.isDebugEnabled())
				logEstimator.debug("arrayErrorSqrts[0]: \r\n" + MatrixUtil.toString(arrayErrorSqrts[0], 7, 3));
			covErrorSqrt = arrayErrorSqrts[0];
		}
		
		AbstractDoubleMatrix sigmaPoints_meanless = MatrixUtil.colSubtract(sigmaPoints, mean);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("sigmaPoints_meanless: \r\n" + MatrixUtil.toString(sigmaPoints_meanless, 7, 3));
		AbstractDoubleMatrix sigmaPoints_multiplied = sigmaPoints_meanless.scalarMultiply(Math.sqrt(wiCov));
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("sigmaPoints_multiplied: \r\n" + MatrixUtil.toString(sigmaPoints_multiplied, 7, 3));
		AbstractDoubleMatrix sigmaPoints_extended = MatrixUtil.colsAppend(
														sigmaPoints_multiplied, 
														covErrorSqrt);

		AbstractDoubleMatrix sigmaPoints_extended_less_first = MatrixUtil.colsLeftTrim(sigmaPoints_extended, 1);

		if (logEstimator.isDebugEnabled())
		{
			logEstimator.debug("sigmaPoints_extended: \r\n" + MatrixUtil.toString(sigmaPoints_extended, 7, 3));
			logEstimator.debug("sigmaPoints_extended_less_first: \r\n" + MatrixUtil.toString(sigmaPoints_extended_less_first, 7, 3));
		}
		AbstractDoubleMatrix [] arrayQR = JSciMatrixMath.qrDecompose((AbstractDoubleMatrix) sigmaPoints_extended_less_first.transpose());
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("arrayQR[1]: \r\n" + MatrixUtil.toString((AbstractDoubleMatrix) arrayQR[1], 7, 3));
		AbstractDoubleMatrix retVal = JSciMatrixMath.cholupdate(arrayQR[1], sigmaPoints_extended, w0cov, 1);
		if (logEstimator.isDebugEnabled())
			logEstimator.debug("retVal+: \r\n" + MatrixUtil.toString(retVal, 7, 3));
		return retVal; 
	}


	public static ModelFilter getModelFilter()
	{
		return new SQUKFModelFilter();
	}
}


class SQUKFModelFilter implements ModelFilter
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

