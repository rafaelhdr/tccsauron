package br.com.r4j.robosim.estimator.impl;

import java.util.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.research.image.sequence.estimator.*;


public class AditiveNosieExtendedKalmanFilter extends BaseEstimator // implements AditiveNosieStatisticalFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieExtendedKalmanFilter.class.getName());
	private static Log logEKF = LogFactory.getLog("ekf");
	private static Log logEKF_I = LogFactory.getLog("ekf_input");
	private static Log logEKF_covar = LogFactory.getLog("ekf_covar");
	private static Log logTime = LogFactory.getLog("time");

	protected Log logEstimator = null;

	private EKFDoubleVectorFunction ekfFunc = null;


	public AditiveNosieExtendedKalmanFilter()
	{
		super();
		logEstimator = logEKF;
	}


	public String getName()
	{
//		return "Filtro de Kalman Extendido";
		return "EKF";
	}


	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException
	{
		if (!(sensModel instanceof EKFDoubleVectorFunction))
			throw new ConfiguratorException("Sensor Model " + sensModel.getName() + " não implementa EKFDoubleVectorFunction");

		super.addSensorModel(sensModel, sens);
	}

	
	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		if (!(dynModel instanceof EKFDoubleVectorFunction))
			throw new ConfiguratorException("Dynamic Model " + dynModel.getName() + " não implementa EKFDoubleVectorFunction");

		this.ekfFunc = (EKFDoubleVectorFunction) dynModel;

		super.setDynamicModel(dynModel, dynModelSensor);
	}

	
	protected void doEstimate()
	{
		if (logEKF.isDebugEnabled())
		{
			log.debug("EKF--------------------------- NOVA ITER: " + this.getIerationCount());
			logEKF.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
			logEKF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
			logEKF.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));
		}

		// predição
		AbstractDoubleVector statei_iminus1 = this.estimateStateMean();
/*
		if (statei_iminus1.dimension() != stateEstimate.dimension())
			this.setCovariance(dynModel.correctCovarByDimension(this.getCovariance()));
//*/

		AbstractDoubleMatrix Fi = ekfFunc.getTransitionMatrixJacobian(this.getMean());
		if (logEKF.isDebugEnabled())
			logEKF.debug("Fi: \r\n" + MatrixUtil.toString(Fi, 9, 4));
		AbstractDoubleSquareMatrix Pi_iminus1 = this.estimateStateCovarNotUsingCovarInc(Fi);
		if (logEKF.isDebugEnabled())
		{
			logEKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 4));
			logEKF.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 9, 4));
		}


		// Verifica quais sensores possuem observacões.
		ArrayList listObsObjects2 = new ArrayList();  
		Iterator itSensModels = listSensModels.iterator();
		Iterator itSens = listSens.iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			Sensor sns = (Sensor) itSens.next();
			EKFDoubleVectorFunction snsFunc = (EKFDoubleVectorFunction) snsModel;

			boolean bResults = sns.hasNewData(snsModel, this);
			if (bResults)
			{
				AbstractDoubleVector vectReadings = new DoubleVector(sns.getDataDimension(snsModel));
				sns.getData(vectReadings, snsModel);
				AbstractDoubleSquareMatrix snsR = sns.getDataCovariance(snsModel);
				listObsObjects2.add(snsModel);
				listObsObjects2.add(vectReadings);
				listObsObjects2.add(snsR);
			}
		}

		AbstractDoubleVector stateEstimateNew = new DoubleVector(dynModel.getDataDimension());
		AbstractDoubleSquareMatrix stateCovarEstimateNew = new DoubleSquareMatrix(dynModel.getDataDimension());
		dynModel.correctByDimension(statei_iminus1, Pi_iminus1, stateEstimateNew, stateCovarEstimateNew);
		statei_iminus1 = stateEstimateNew;
		Pi_iminus1 = stateCovarEstimateNew;
		Pi_iminus1 = Pi_iminus1.add(dynModel.getModelIncrementalCovariance(null, null, dynModelSensor.getDataCovariance(dynModel)));
		if (logEKF.isDebugEnabled())
			logEKF.debug("dynModel.getModelIncrementalCovariance: \r\n" + MatrixUtil.toString(dynModel.getModelIncrementalCovariance(null, null, dynModelSensor.getDataCovariance(dynModel)), 9, 4));

		if (logEstimator.isDebugEnabled())
			logEstimator.debug("dynModel.getDataDimension(): " + dynModel.getDataDimension());

		this.setPredictedState(statei_iminus1, Pi_iminus1);

		// Verifica quais modelos de sensores possuem observacões.
		ArrayList listObsObjects = new ArrayList();  
		Iterator itObsObjects = listObsObjects2.iterator();
		while (itObsObjects.hasNext())
		{
			SensorModel snsModel = (SensorModel) itObsObjects.next();
			AbstractDoubleVector vectReadings = (AbstractDoubleVector) itObsObjects.next();
			AbstractDoubleSquareMatrix snsR = (AbstractDoubleSquareMatrix) itObsObjects.next();
			EKFDoubleVectorFunction snsFunc = (EKFDoubleVectorFunction) snsModel;

			boolean bResults = snsFunc.canProduceObservations(vectReadings, snsR, statei_iminus1, Pi_iminus1);
			if (bResults)
			{
				AbstractDoubleVector vectObservations = snsModel.getObservation(vectReadings);
				AbstractDoubleSquareMatrix R = snsModel.getObservationCovariance(snsR);
				if (logEstimator.isDebugEnabled())
					logEstimator.debug("R: \r\n" + MatrixUtil.toString(R, 9, 4));
				
				listObsObjects.add(snsModel);
				listObsObjects.add(vectObservations);
				listObsObjects.add(R);
			}
		}

		itObsObjects = listObsObjects.iterator();
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
			}

			EKFDoubleVectorFunction snsFunc = (EKFDoubleVectorFunction) snsModel;
			AbstractDoubleVector vectObservations = (AbstractDoubleVector) itObsObjects.next();
			AbstractDoubleSquareMatrix R = (AbstractDoubleSquareMatrix) itObsObjects.next();
			if (logEstimator.isDebugEnabled())
			{
				logEstimator.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 4));
				logEstimator.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 9, 4));
				logEstimator.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 9, 4));
				logEstimator.debug("R: \r\n" + MatrixUtil.toString(R, 9, 4));
			}

			AbstractDoubleVector vectObsPredicted = snsFunc.produceResults(statei_iminus1, vectObservations, Pi_iminus1);
			AbstractDoubleMatrix Hi = snsFunc.getTransitionMatrixJacobian(statei_iminus1);
			if (logEKF.isDebugEnabled())
				logEKF.debug("Hi: \r\n" + MatrixUtil.toString(Hi, 9, 4));

			AbstractDoubleMatrix Kfi = this.calculateGain(Hi, Pi_iminus1, snsModel, R);
			if (logEKF.isDebugEnabled())
				logEKF.debug("Kfi: \r\n" + MatrixUtil.toString(Kfi, 9, 4));
			AbstractDoubleVector statei_i = this.filterStateMean(Kfi, statei_iminus1, vectObsPredicted, snsModel, vectObservations);
			AbstractDoubleSquareMatrix Pi_i = this.filterStateCovar(Kfi, Hi, Pi_iminus1);

			if (logEKF.isDebugEnabled())
			{
				logEKF.debug("statei_i: \r\n" + MatrixUtil.toString(statei_i, 9, 4));
				logEKF.debug("Pi_i: \r\n" + MatrixUtil.toString(Pi_i, 9, 4));
			}

			this.setState(statei_i, Pi_i);
			statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;

			if (logEstimator.isDebugEnabled())
			{
				AbstractDoubleVector obsPredAfter = ((EKFDoubleVectorFunction) snsFunc).produceResults(statei_i, vectObservations, Pi_iminus1);
				logEstimator.debug("obs pred corr: \r\n" + MatrixUtil.toString(obsPredAfter, 9, 4));
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

		if (logEKF.isDebugEnabled())
		{
			log.debug("END EKF--------------------------- ITER: " + this.getIerationCount());
			logEKF.debug("END EKF--------------------------- ITER: " + this.getIerationCount());
		}
	}

	
	private AbstractDoubleVector estimateStateMean()
	{
		AbstractDoubleVector vectReadings = new DoubleVector(dynModelSensor.getDataDimension(dynModel));
		dynModelSensor.getData(vectReadings, dynModel);
		if (logEKF.isDebugEnabled())
			logEKF.debug("vectReadings: \r\n" + MatrixUtil.toString(vectReadings, 9, 4));
		return dynModel.produceResults(this.getMean(), vectReadings, this.getCovariance());
	}


	private AbstractDoubleSquareMatrix estimateStateCovarNotUsingCovarInc(AbstractDoubleMatrix Fi)
	{
		AbstractDoubleMatrix FiT = (AbstractDoubleMatrix) Fi.transpose();

		AbstractDoubleMatrix tmp1 = Fi.multiply(this.getCovariance()).multiply(FiT);
//		tmp1 = tmp1.add(dynModel.getModelIncrementalCovariance(null, null, dynModelSensor.getDataCovariance(dynModel)));
//		if (logEKF.isDebugEnabled())
//			logEKF.debug("dynModel.getModelIncrementalCovariance: \r\n" + MatrixUtil.toString(dynModel.getModelIncrementalCovariance(null, null, dynModelSensor.getDataCovariance(dynModel)), 9, 4));

		return MatrixUtil.convert2SquareMatrix(tmp1);
	}


	private AbstractDoubleMatrix calculateGain(AbstractDoubleMatrix Hi, AbstractDoubleSquareMatrix stateCovarEstimate, SensorModel snsModel, AbstractDoubleSquareMatrix R)
	{
		if (logEKF.isDebugEnabled())
			logEKF.debug("snsModel.getObservationCovariance: \r\n" + MatrixUtil.toString(R, 9, 4));

		AbstractDoubleMatrix HiT = (AbstractDoubleMatrix) Hi.transpose();
/*
		{

			AbstractDoubleSquareMatrix Ptmp = MatrixUtil.diagonal(stateCovarEstimate);
			AbstractDoubleMatrix matDivisor1 = Hi.multiply(Ptmp).multiply(HiT);
			if (logEKF.isDebugEnabled())
				logEKF.debug("Ptmp (v_diag): \r\n" + MatrixUtil.toString(Ptmp, 9, 4));
			if (logEKF.isDebugEnabled())
				logEKF.debug("P em R (v_diag): \r\n" + MatrixUtil.toString(matDivisor1, 9, 4));
			AbstractDoubleMatrix matDivisor = matDivisor1.add(R);
			AbstractDoubleMatrix matDivisorInv = MatrixUtil.convert2SquareMatrix(matDivisor).inverse();
			if (logEKF.isDebugEnabled())
				logEKF.debug("matDivisorInv (v_diag): \r\n" + MatrixUtil.toString(matDivisorInv, 9, 4));
			if (logEKF.isDebugEnabled())
				logEKF.debug("stateCovarEstimate.multiply(HiT) (v_diag): \r\n" + MatrixUtil.toString(Ptmp.multiply(HiT), 9, 4));
			if (logEKF.isDebugEnabled())
				logEKF.debug("gain (v_diag): \r\n" + MatrixUtil.toString(stateCovarEstimate.multiply(HiT).multiply(matDivisorInv), 9, 4));
			return stateCovarEstimate.multiply(HiT).multiply(matDivisorInv);
		}
/*/
		AbstractDoubleMatrix matDivisor1 = Hi.multiply(stateCovarEstimate).multiply(HiT);
		if (logEKF.isDebugEnabled())
			logEKF.debug("P em R: \r\n" + MatrixUtil.toString(matDivisor1, 9, 4));
		AbstractDoubleMatrix matDivisor = matDivisor1.add(R);
		AbstractDoubleMatrix matDivisorInv = MatrixUtil.convert2SquareMatrix(matDivisor).inverse();
		if (logEKF.isDebugEnabled())
			logEKF.debug("matDivisorInv: \r\n" + MatrixUtil.toString(matDivisorInv, 9, 4));
		if (logEKF.isDebugEnabled())
			logEKF.debug("stateCovarEstimate.multiply(HiT): \r\n" + MatrixUtil.toString(stateCovarEstimate.multiply(HiT), 9, 4));
		return stateCovarEstimate.multiply(HiT).multiply(matDivisorInv);
//*/
	}


	private AbstractDoubleVector filterStateMean(AbstractDoubleMatrix gain, AbstractDoubleVector stateMeanEstimate, AbstractDoubleVector vectObsPredicted, SensorModel snsModel, AbstractDoubleVector vectObservations)
	{
		if (logEKF.isDebugEnabled())
		{
			logEKF.debug("vectObsPredicted: \r\n" + MatrixUtil.toString(vectObsPredicted, 9, 4));
			logEKF.debug("vectObservations: \r\n" + MatrixUtil.toString(vectObservations, 9, 4));
		}

		AbstractDoubleVector measDiff = vectObservations.subtract(vectObsPredicted);
		AbstractDoubleVector measGain = gain.multiply(measDiff);
		if (logEKF.isDebugEnabled())
			logEKF.debug("measGain: \r\n" + MatrixUtil.toString(measGain, 9, 4));

		return stateMeanEstimate.add(measGain);
	}


	private AbstractDoubleSquareMatrix filterStateCovar(AbstractDoubleMatrix gain, AbstractDoubleMatrix Hi, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		AbstractDoubleSquareMatrix matIdentity = JSciMatrixMath.getIdentity(stateCovarEstimate.rows());
		return MatrixUtil.convert2SquareMatrix(matIdentity.subtract(gain.multiply(Hi)).multiply(stateCovarEstimate));
	}


	public static ModelFilter getModelFilter()
	{
		return new EKFModelFilter();
	}
}


class EKFModelFilter implements ModelFilter
{
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return (dynModel instanceof EKFDoubleVectorFunction);
	}


	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return (sensModel instanceof EKFDoubleVectorFunction);
	}
}

/*
000800992.7183	000966042.3748	002867509.5654
000966042.3748	001165101.5839	003458378.4670
002867509.5654	003458378.4670	010265529.5103
|  2804.159 -2896.201   192.411 |
| -2896.201  2994.606  -199.853 |
|   192.411  -199.853    13.582 |


000800992.9183	000966042.3748	002867509.5654
000966042.3748	001165101.7839	003458378.4670
002867509.5654	003458378.4670	010265529.7103
|  3.038 -1.908 -0.206 |
| -1.908  3.115 -0.516 |
| -0.206 -0.516  0.231 |


000800993.9183	000966042.3748	002867509.5654
000966042.3748	001165102.7839	003458378.4670
002867509.5654	003458378.4670	010265530.7103
|  0.665 -0.172 -0.128 |
| -0.172  0.656 -0.173 |
| -0.128 -0.173  0.094 |


//*/