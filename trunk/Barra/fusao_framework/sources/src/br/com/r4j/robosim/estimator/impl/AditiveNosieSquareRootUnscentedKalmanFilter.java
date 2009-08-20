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


public class AditiveNosieSquareRootUnscentedKalmanFilter extends BaseAditiveUnscentedKalmanFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieSquareRootUnscentedKalmanFilter.class.getName());
	private static Log logUKF = LogFactory.getLog("srukf");
	private static Log logUKF_I = LogFactory.getLog("ukf_input");
	private static Log logUKF_covar = LogFactory.getLog("ukf_covar");
	private static Log logTime = LogFactory.getLog("time");

	private AbstractDoubleMatrix S = null;


	public AditiveNosieSquareRootUnscentedKalmanFilter()
	{
		super();
	}


	public String getName()
	{
		return "Filtro de Kalman Unscented Square Root";
	}


	public void setState(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		// Inicia.
		if (S == null)
			S = JSciMatrixMath.choleskyDecompose(stateCovarEstimate)[0];

		super.setState(stateEstimate, stateCovarEstimate);
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

		this.setModifiers(stateEstimate.dimension());
		logUKF.debug("w0state = " + w0state + ", w0cov = " + w0cov + ", wi = " + wi + ", gamma: " + gamma + ", delta = " + delta);

		logUKF.debug("S: \r\n" + MatrixUtil.toString(S, 9, 4));
		AbstractDoubleMatrix stateLastSigma = this.generateSigmaPointsWithSqrtCovar(stateEstimate, S, gamma);

		// Calcula os sigmas pela tranformação de estado
		AbstractDoubleMatrix sigmaStatei_iminus1 = new DoubleMatrix(stateLastSigma.rows(), stateLastSigma.columns());
		stateFunction.produceResults(stateLastSigma, vectDynReadings, new DoubleMatrix(stateLastSigma.rows(), stateLastSigma.columns()), sigmaStatei_iminus1, this.getCovariance());
		logUKF.debug("sigmaStatei_iminus1: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1, 9, 7));

		// 2 - Calcula os vetores ...
		AbstractDoubleVector statei_iminus1 = this.calculateMean(sigmaStatei_iminus1, w0state, wi);
		logUKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 7));

		double detQ = JSciMatrixMath.det(Q);
		logUKF.debug("detQ: " + detQ);
		if (!Double.isNaN(detQ) && detQ < 0)
		{
			logUKF.debug("detQ < 0!!!");
			throw new EstimationException("SRUKF: detQ < 0!!!. Não pode estimar.");
		}
		else if (detQ == 0 || Math.abs(detQ) < 1E-9 || Double.isNaN(detQ))
		{
			int countIt = 0;
			double minDiag = MatrixUtil.getAbsMinValueOnDiagonal(Q);
			if (!Double.isNaN(detQ) && detQ > 0)
			{
				minDiag = detQ / (1E-9) / Q.rows();
			}
			else if (minDiag < 1E-3)
				minDiag = 1E-3;
			while (detQ == 0 || Math.abs(detQ) < 1E-9 || Double.isNaN(detQ))
			{ 
				MatrixUtil.sumOnDiagonal(Q, minDiag);
				detQ = JSciMatrixMath.det(Q);
				logUKF.debug("it: detQ: " + detQ);
				
				if (countIt++ > 200)
				{
					logUKF.debug("Kabum!!!"); logUKF.debug("Q: \r\n" + MatrixUtil.toString(Q, 9, 7));
					throw new EstimationException("SRUKF: Kabum!!!. countIt > 200. Não pode estimar.");
				}
			}
		}

		S = this.getSi_minus1(sigmaStatei_iminus1, statei_iminus1, Q, w0cov, wi);

		AbstractDoubleSquareMatrix Pi_iminus1 = MatrixUtil.convert2SquareMatrix(S.multiply((AbstractDoubleMatrix) S.transpose()));
		logUKF.debug("Pi_iminus1.isSymmetric(): " + Pi_iminus1.isSymmetric() + ", Pi_iminus1.det(): " + JSciMatrixMath.det(Pi_iminus1) + "Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 9, 4));
		this.setPredictedState(statei_iminus1, Pi_iminus1);

		Iterator itObsObjects = listObsObjects.iterator();
		while (itObsObjects.hasNext())
		{
			SensorModel snsModel = (SensorModel) itObsObjects.next();
			logUKF.debug("snsModel.getName(): " + snsModel.getName());
			UKFDoubleVectorFunction snsFunc = (UKFDoubleVectorFunction) snsModel;
			AbstractDoubleVector vectObservations = (AbstractDoubleVector) itObsObjects.next();
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) itObsObjects.next();
			
			// Calcula os sigmas pela tranformação de medição
			DoubleMatrix obsPredSigma = new DoubleMatrix(vectObservations.dimension(), sigmaStatei_iminus1.columns());
			logUKF.debug("sigmaStatei_iminus1: \r\n" + MatrixUtil.toString(sigmaStatei_iminus1, 9, 7));
			snsFunc.produceResults(sigmaStatei_iminus1, statei_iminus1, new DoubleMatrix(vectObservations.dimension(), sigmaStatei_iminus1.columns()), obsPredSigma, Pi_iminus1);
			logUKF.debug("obsPredSigma: \r\n" + MatrixUtil.toString(obsPredSigma, 9, 7));

			AbstractDoubleVector obsPred = this.calculateMean(obsPredSigma, w0state, wi);
			AbstractDoubleMatrix Sy = this.getSi_minus1(obsPredSigma, obsPred, R, w0cov, wi);

			// Precisa trocar pelo squares ...
			AbstractDoubleSquareMatrix Pyy = MatrixUtil.convert2SquareMatrix(Sy.multiply((AbstractDoubleMatrix) Sy.transpose()));
			AbstractDoubleMatrix PyyInv = Pyy.inverse();
			logUKF.debug("PyyInv: \r\n" + MatrixUtil.toString(PyyInv, 9, 4));
			
			AbstractDoubleMatrix pXY = this.calculateCov(sigmaStatei_iminus1, statei_iminus1, obsPredSigma, obsPred, w0cov, wi);
			logUKF.debug("pXY: \r\n" + MatrixUtil.toString(pXY, 9, 4));
			
			AbstractDoubleMatrix gain = pXY.multiply(PyyInv);
			logUKF.debug("gain: \r\n" + MatrixUtil.toString(gain, 9, 4));
			
			AbstractDoubleMatrix U = gain.multiply(Sy);
			logUKF.debug("U: \r\n" + MatrixUtil.toString(U, 9, 7));
			
			// Teste ...
			{
				AbstractDoubleMatrix S2 = S.multiply((AbstractDoubleMatrix) S.transpose());
				AbstractDoubleMatrix U2 = U.multiply((AbstractDoubleMatrix) U.transpose());
				AbstractDoubleMatrix S2mU2 = S2.subtract(U2);
				logUKF.debug("S2: \r\n" + MatrixUtil.toString(S2, 9, 7));
				logUKF.debug("U2: \r\n" + MatrixUtil.toString(U2, 9, 7));
				logUKF.debug("S2mU2: \r\n" + MatrixUtil.toString(S2mU2, 9, 7));

				logUKF.debug("S(B): \r\n" + MatrixUtil.toString(S, 9, 7));
				S = (AbstractDoubleMatrix) JSciMatrixMath.cholupdate((AbstractDoubleMatrix) S.transpose(), U, -1).transpose();			
				logUKF.debug("S(A): \r\n" + MatrixUtil.toString(S, 9, 7));
			}

			AbstractDoubleSquareMatrix Pi_i = MatrixUtil.convert2SquareMatrix(S.multiply((AbstractDoubleMatrix) S.transpose()));
			logUKF.debug("Pi_i: \r\n" + MatrixUtil.toString(Pi_i, 9, 7));

			AbstractDoubleVector obsDiff = vectObservations.subtract(obsPred);
			AbstractDoubleVector stateGain =  gain.multiply(obsDiff);
			AbstractDoubleVector statei_i = statei_iminus1.add(stateGain);

			logUKF.debug("stateGain : \r\n" + MatrixUtil.toString(stateGain, 9, 7));
			logUKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 7));
			logUKF.debug("statei_i: \r\n" + MatrixUtil.toString(statei_i, 9, 7));

			this.setState(statei_i, Pi_i);
			statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;
		}
	}


	/**
	 * @param sigmaStatei_iminus1
	 * @param statei_iminus1
	 * @param Q
	 * @return
	 */
	private AbstractDoubleMatrix getSi_minus1(AbstractDoubleMatrix sigmaPoints, AbstractDoubleVector mean, AbstractDoubleSquareMatrix covError, double w0, double wi)
	{
		AbstractDoubleMatrix [] arrayErrorSqrts = JSciMatrixMath.choleskyDecompose(covError);
		logUKF.debug("arrayErrorSqrts[0]: \r\n" + MatrixUtil.toString(arrayErrorSqrts[0], 9, 4));
		
		AbstractDoubleMatrix sigmaPoints_meanless = MatrixUtil.colSubtract(sigmaPoints, mean);
		logUKF.debug("sigmaPoints_meanless: \r\n" + MatrixUtil.toString(sigmaPoints_meanless, 9, 4));
		AbstractDoubleMatrix sigmaPoints_multiplied = sigmaPoints_meanless.scalarMultiply(Math.sqrt(wi));
		logUKF.debug("sigmaPoints_multiplied: \r\n" + MatrixUtil.toString(sigmaPoints_multiplied, 9, 4));
		AbstractDoubleMatrix sigmaPoints_extended = MatrixUtil.colsAppend(
														sigmaPoints_multiplied, 
														arrayErrorSqrts[0]);
		logUKF.debug("sigmaPoints_extended: \r\n" + MatrixUtil.toString(sigmaPoints_extended, 9, 4));
		AbstractDoubleMatrix [] arrayQR = JSciMatrixMath.qrDecompose((AbstractDoubleMatrix) sigmaPoints_extended.transpose());
		AbstractDoubleMatrix retVal = (AbstractDoubleMatrix) arrayQR[1].transpose();
		retVal = JSciMatrixMath.cholupdate((AbstractDoubleMatrix) retVal.transpose(), sigmaPoints, w0, 1);			
		logUKF.debug("retVal+: \r\n" + MatrixUtil.toString(retVal, 9, 4));
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

