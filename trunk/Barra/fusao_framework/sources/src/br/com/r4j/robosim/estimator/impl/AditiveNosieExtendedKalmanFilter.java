package br.com.r4j.robosim.estimator.impl;

import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;


public class AditiveNosieExtendedKalmanFilter extends BaseEstimator // implements AditiveNosieStatisticalFilter
{
	private static Log log = LogFactory.getLog(AditiveNosieExtendedKalmanFilter.class.getName());
	private static Log logEKF = LogFactory.getLog("ekf");
	private static Log logEKF_I = LogFactory.getLog("ekf_input");
	private static Log logEKF_covar = LogFactory.getLog("ekf_covar");
	private static Log logTime = LogFactory.getLog("time");

	private EKFDoubleVectorFunction ekfFunc = null;


	public AditiveNosieExtendedKalmanFilter()
	{
		super();
	}


	public String getName()
	{
		return "Filtro de Kalman Extendido";
	}


	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException
	{
		if (!(sensModel instanceof EKFDoubleVectorFunction))
			throw new ConfiguratorException("Sensor Model " + sensModel.getName() + " n�o implementa EKFDoubleVectorFunction");

		super.addSensorModel(sensModel, sens);
	}

	
	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		if (!(dynModel instanceof EKFDoubleVectorFunction))
			throw new ConfiguratorException("Dynamic Model " + dynModel.getName() + " n�o implementa EKFDoubleVectorFunction");

		this.ekfFunc = (EKFDoubleVectorFunction) dynModel;

		super.setDynamicModel(dynModel, dynModelSensor);
	}

	
	protected void doEstimate()
	{
		logEKF.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
		logEKF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
		logEKF.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));

		// predi��o
		AbstractDoubleVector statei_iminus1 = this.estimateStateMean();
		AbstractDoubleMatrix Fi = ekfFunc.getTransitionMatrixJacobian(this.getMean());
		logEKF.debug("Fi: \r\n" + MatrixUtil.toString(Fi, 9, 4));
		AbstractDoubleSquareMatrix Pi_iminus1 = this.estimateStateCovar(Fi);
		logEKF.debug("statei_iminus1: \r\n" + MatrixUtil.toString(statei_iminus1, 9, 4));
		logEKF.debug("Pi_iminus1: \r\n" + MatrixUtil.toString(Pi_iminus1, 9, 4));
		this.setPredictedState(statei_iminus1, Pi_iminus1);

		Iterator itSensModels = listSensModels.iterator();
		Iterator itSens = listSens.iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			Sensor sns = (Sensor) itSens.next();
			EKFDoubleVectorFunction snsFunc = (EKFDoubleVectorFunction) snsModel;

			boolean bResults = sns.hasNewData();
			if (bResults)
			{
				AbstractDoubleVector vectReadings = new DoubleVector(sns.getDataDimension());
				sns.getData(vectReadings);
				AbstractDoubleSquareMatrix snsR = sns.getDataCovariance();
				bResults = snsModel.canProduceObservations(vectReadings, snsR);
				if (bResults)
				{
					AbstractDoubleVector vectObsPredicted = new DoubleVector(snsModel.getDataDimension());
					snsModel.produceResults(statei_iminus1, vectObsPredicted, Pi_iminus1);
					AbstractDoubleMatrix Hi = snsFunc.getTransitionMatrixJacobian(statei_iminus1);
					logEKF.debug("Hi: \r\n" + MatrixUtil.toString(Hi, 9, 4));
	
					AbstractDoubleMatrix Kfi = this.calculateGain(Hi, Pi_iminus1, snsModel, sns, snsR);
					logEKF.debug("Kfi: \r\n" + MatrixUtil.toString(Kfi, 9, 4));
					AbstractDoubleVector statei_i = this.filterStateMean(Kfi, statei_iminus1, vectObsPredicted, snsModel, sns, vectReadings);
					AbstractDoubleSquareMatrix Pi_i = this.filterStateCovar(Kfi, Hi, Pi_iminus1);
	
					logEKF.debug("statei_i: \r\n" + MatrixUtil.toString(statei_i, 9, 4));
					logEKF.debug("Pi_i: \r\n" + MatrixUtil.toString(Pi_i, 9, 4));
	
					this.setState(statei_i, Pi_i);
					statei_iminus1 = statei_i; Pi_iminus1 = Pi_i;
				}
			}
		}
	}

	
	private AbstractDoubleVector estimateStateMean()
	{
		AbstractDoubleVector vectReadings = new DoubleVector(dynModelSensor.getDataDimension());
		AbstractDoubleVector vectEst = new DoubleVector(dynModel.getDataDimension());
		dynModelSensor.getData(vectReadings);
		dynModel.produceResults(this.getMean(), vectReadings, this.getCovariance(), vectEst);

		return vectEst;
	}


	private AbstractDoubleSquareMatrix estimateStateCovar(AbstractDoubleMatrix Fi)
	{
		AbstractDoubleMatrix FiT = (AbstractDoubleMatrix) Fi.transpose();

		AbstractDoubleMatrix tmp1 = Fi.multiply(this.getCovariance()).multiply(FiT);
		tmp1 = tmp1.add(dynModel.getModelIncrementalCovariance(null, null, dynModelSensor.getDataCovariance()));
		logEKF.debug("dynModel.getModelIncrementalCovariance: \r\n" + MatrixUtil.toString(dynModel.getModelIncrementalCovariance(null, null, dynModelSensor.getDataCovariance()), 9, 4));

		return MatrixUtil.convert2SquareMatrix(tmp1);
	}


	private AbstractDoubleMatrix calculateGain(AbstractDoubleMatrix Hi, AbstractDoubleSquareMatrix stateCovarEstimate, SensorModel snsModel, Sensor snsModelSensor, AbstractDoubleSquareMatrix snsR)
	{
		AbstractDoubleSquareMatrix R = snsModel.getObservationCovariance(snsR);
		logEKF.debug("snsModel.getObservationCovariance: \r\n" + MatrixUtil.toString(R, 9, 4));

		AbstractDoubleMatrix HiT = (AbstractDoubleMatrix) Hi.transpose();
		AbstractDoubleMatrix matDivisor = Hi.multiply(stateCovarEstimate).multiply(HiT).add(R);
		AbstractDoubleMatrix matDivisorInv = MatrixUtil.convert2SquareMatrix(matDivisor).inverse();
		return stateCovarEstimate.multiply(HiT).multiply(matDivisorInv);
	}


	private AbstractDoubleVector filterStateMean(AbstractDoubleMatrix gain, AbstractDoubleVector stateMeanEstimate, AbstractDoubleVector vectObsPredicted, SensorModel snsModel, Sensor snsModelSensor, AbstractDoubleVector vectReadings)
	{
		AbstractDoubleVector vectObservations = snsModel.getObservation(vectReadings);

		logEKF.debug("vectReadings: \r\n" + MatrixUtil.toString(vectReadings, 9, 4));
		logEKF.debug("vectObsPredicted: \r\n" + MatrixUtil.toString(vectObsPredicted, 9, 4));
		AbstractDoubleVector measDiff = vectObservations.subtract(vectObsPredicted);
		AbstractDoubleVector measGain = gain.multiply(measDiff);
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

