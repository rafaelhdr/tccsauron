package br.com.r4j.robosim.estimator.impl;

import java.util.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleMatrix;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.math.JSciMatrixMath;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EKFDoubleVectorFunction;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.ModelFilter;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.*;
import br.com.r4j.robosim.RobotParticles;


public class AditiveNosieParticleFilter extends BaseEstimator implements ParticleFilter, Configurable
{
	private static Log log = LogFactory.getLog(AditiveNosieParticleFilter.class.getName());
	private static Log logPF = LogFactory.getLog("pf");
	private static Log logPF_I = LogFactory.getLog("pf_input");
	private static Log logPF_covar = LogFactory.getLog("pf_covar");
	private static Log logTime = LogFactory.getLog("time");

	
	private ParticleCloudPredictorFunction stateFunction = null;

	private AbstractDoubleVector weight = null;
	private AbstractDoubleMatrix state = null;
	private AbstractDoubleMatrix stateNext = null;

	private double resamplingMinFactor = -1;
	private int numberOfParticles = -1;

	private ArrayList listRndr = null;
	private RobotParticles partRndr = null;


	public AditiveNosieParticleFilter()
	{
		resamplingMinFactor = 0.1;
		numberOfParticles = 60;

		partRndr = new RobotParticles(this);
		listRndr = new ArrayList();
		listRndr.add(partRndr);
		this.addRobotTracker(partRndr);
	}


	public String getName()
	{
		return "Filtro de Partículas";
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		log.debug("configure configure configure configure configure");
		if (props.containsProperty(strBaseKey + "/resamplingMinFactor"))
			resamplingMinFactor = props.getDoubleProperty(strBaseKey + "/resamplingMinFactor").doubleValue();
		if (props.containsProperty(strBaseKey + "/numberOfParticles"))
			numberOfParticles = props.getIntegerProperty(strBaseKey + "/numberOfParticles").intValue();
	}


	public List getRenderers()
	{
		return listRndr;
	}


	public void setState(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		if (weight == null)
		{
			weight = new DoubleVector(numberOfParticles);
			state = new DoubleMatrix(numberOfParticles, stateEstimate.dimension());
			stateNext = new DoubleMatrix(numberOfParticles, stateEstimate.dimension());

			for (int i = 0; i < weight.dimension(); i++)
				weight.setComponent(i, 1.0/weight.dimension());
			for (int i = 0; i < state.rows(); i++) for (int j = 0; j < state.columns(); j++)
				state.setElement(i, j, stateEstimate.getComponent(j));
		}
		super.setState(stateEstimate, stateCovarEstimate);
	}


	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException
	{
		if (!(sensModel instanceof PosteriorProbabilityDensityFunction))
			throw new ConfiguratorException("Sensor Model " + sensModel.getName() + " não implementa AditiveNoisePosteriorProbabilityDensityFunction");

		super.addSensorModel(sensModel, sens);
	}


	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		if (!(dynModel instanceof ParticleCloudPredictorFunction))
			throw new ConfiguratorException("Dynamic Model " + dynModel.getName() + " não implementa AdditiveNoiseParticleCloudPredictorFunction");

		stateFunction = (ParticleCloudPredictorFunction) dynModel;

		super.setDynamicModel(dynModel, dynModelSensor);
	}

	
	protected void doEstimate()
	{
		logPF.debug("------------------------------ NOVA ITER: " + this.getIerationCount());
		logPF.debug("stateEstimate: \r\n" + MatrixUtil.toString(stateEstimate, 9, 4));
		logPF.debug("stateCovarEstimate: \r\n" + MatrixUtil.toString(stateCovarEstimate, 9, 4));

		// 1- Realiza a predição das particulas.
		AbstractDoubleVector vectEstReadings = new DoubleVector(dynModelSensor.getDataDimension());
		dynModelSensor.getData(vectEstReadings);
		stateFunction.calculateNextStateCloud(state, 
											stateNext, 
											vectEstReadings, 
//											dynModel.getModelIncrementalCovariance(stateEstimate, vectEstReadings, dynModelSensor.getDataCovariance()));
											dynModelSensor.getDataCovariance());
		logPF.debug("weight: \r\n" + MatrixUtil.toString(weight, 9, 4));
		logPF.debug("vectEstReadings: \r\n" + MatrixUtil.toString(vectEstReadings, 9, 4));
//		logPF.debug("stateNext: \r\n" + MatrixUtil.toString(stateNext, 9, 4));

		// 2 - Realiza o update dos pesos.
/*		
		double cteIni = (double) (1000L*listSensModels.size());
		for (int i = 0; i < weight.dimension(); i++)
			weight.setComponent(i, weight.getComponent(i)*cteIni);
//*/
		double wiSum = 0;
		for (int i = 0; i < weight.dimension(); i++)
			wiSum += weight.getComponent(i);
		
		if (wiSum > 0)
		{
			for (int i = 0; i < weight.dimension(); i++)
				weight.setComponent(i, weight.getComponent(i)/wiSum);
		}
		else
		{
			logPF.debug("wiSum <= 0: " + wiSum);
			for (int i = 0; i < weight.dimension(); i++)
				weight.setComponent(i, 1.0/weight.dimension());
		}

		// 2.5 - Calcula o valor médio e covariância das partículas.
		this.calculateStateMean(); this.calculateStateCovar();
		this.setPredictedState(stateEstimate, stateCovarEstimate);

		Iterator itSensModels = listSensModels.iterator(), itSens = listSens.iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			Sensor sns = (Sensor) itSens.next();
			PosteriorProbabilityDensityFunction snsFunc = (PosteriorProbabilityDensityFunction) snsModel;

			boolean bResults = sns.hasNewData();
			if (bResults)
			{
				AbstractDoubleVector vectReadings = new DoubleVector(sns.getDataDimension());
				sns.getData(vectReadings);
				AbstractDoubleSquareMatrix snsR = sns.getDataCovariance();
				bResults = snsModel.canProduceObservations(vectReadings, snsR);
				if (bResults)
				{
					AbstractDoubleSquareMatrix R = snsModel.getObservationCovariance(snsR);
					snsFunc.setAdditiveNoise(R);

					AbstractDoubleVector vectObservations = snsModel.getObservation(vectReadings);
					snsFunc.setObservations(vectObservations);

					for (int i = 0; i < weight.dimension(); i++)
					{
						double wi = weight.getComponent(i)*snsFunc.stateProbability(stateNext, i);
						weight.setComponent(i, weight.getComponent(i) * wi);
					}
				}
			}
		}

		wiSum = 0;
		for (int i = 0; i < weight.dimension(); i++)
			wiSum += weight.getComponent(i);

		if (wiSum > 0)
		{
			for (int i = 0; i < weight.dimension(); i++)
				weight.setComponent(i, weight.getComponent(i)/wiSum);
		}
		else
		{
			logPF.debug("wiSum <= 0: " + wiSum);
			for (int i = 0; i < weight.dimension(); i++)
				weight.setComponent(i, 1.0/weight.dimension());
		}

		// 3 - Calcula o valor médio e covariância das partículas.
		this.calculateStateMean(); this.calculateStateCovar();
		this.setState(stateEstimate, stateCovarEstimate);

		// 4 - Verifica se há a necessidade de realizar ressampling.
		if (this.needResampling())
		{
			logPF.debug("resampling ...");
			this.resample();
		}
		else
		{
			AbstractDoubleMatrix tmp = state; state = stateNext; stateNext = tmp;
		}
	}


	protected boolean needResampling()
	{
		double cvi = 0;
		for (int i = 0; i < weight.dimension(); i++)
		{
			double factor = weight.getComponent(i)*weight.dimension() - 1;
			cvi += factor*factor;
		}
		cvi = cvi/weight.dimension();
		double ESS = weight.dimension()/(1 + cvi);
		return (ESS < weight.dimension()*resamplingMinFactor);
	}


	protected void calculateStateMean()
	{
		for (int i = 0; i < state.columns(); i++) 
		{
			double acc = 0;
			for (int j = 0; j < state.rows(); j++)
				acc += weight.getComponent(j)*stateNext.getElement(j, i);
			stateEstimate.setComponent(i, acc);
		}
	}


	protected void calculateStateCovar()
	{
		AbstractDoubleMatrix xDiff = new DoubleMatrix(state.columns(), 1);
		AbstractDoubleMatrix covarSum = null;
		double w2Sum = 0;
		for (int i = 0; i < state.rows(); i++) 
		{
			double w2 = weight.getComponent(i)*weight.getComponent(i);
			w2Sum += w2;
			for (int j = 0; j < state.columns(); j++)
				xDiff.setElement(j, 0, stateNext.getElement(i, j) - stateEstimate.getComponent(j));
			AbstractDoubleMatrix xDiffT = (AbstractDoubleMatrix) xDiff.transpose();
			AbstractDoubleMatrix covarPar = xDiff.multiply(xDiffT);
			covarPar = covarPar.scalarMultiply(w2);
			if (covarSum == null)
				covarSum = covarPar;
			else
				covarSum = covarSum.add(covarPar);
		}
		double covarCte = 1.0 * state.rows() / (state.rows() - 1) / w2Sum;
		stateCovarEstimate = MatrixUtil.convert2SquareMatrix(covarSum.scalarMultiply(covarCte));
	}


	protected void resample()
	{
		logPF.debug("resampling ...");
		DoubleVector accWeight = new DoubleVector(weight.dimension());
		DoubleVector accRnd = new DoubleVector(weight.dimension());
		accWeight.setComponent(0, weight.getComponent(0));
		accRnd.setComponent(0, -Math.log(Math.random()));
		for (int i = 1; i < weight.dimension(); i++)
		{
			accWeight.setComponent(i, weight.getComponent(i) + accWeight.getComponent(i-1));
			accRnd.setComponent(i, -Math.log(Math.random()) + accRnd.getComponent(i-1));
		}
		accWeight.setComponent(weight.dimension()-1, 1);

		for (int i = 0; i < weight.dimension(); i++)
		{
			weight.setComponent(i, 1.0/weight.dimension());
			accRnd.setComponent(i, accRnd.getComponent(i)/accRnd.getComponent(weight.dimension()-1));
		}

		for (int i = 0, j = 0; i < weight.dimension(); )
		{
			if (accRnd.getComponent(i) <= accWeight.getComponent(j))
			{
				for (int k = 0; k < state.columns(); k++)
					state.setElement(i, k, stateNext.getElement(j, k));
					
				Iterator itSensModels = listSensModels.iterator(), itSens = listSens.iterator();
				while (itSensModels.hasNext())
				{
					PosteriorProbabilityDensityFunction snsFunc = (PosteriorProbabilityDensityFunction) itSensModels.next();
					snsFunc.copyParticleData(j, i);
				}
				i++;
			}
			else
				j++;
		}
	}


	public AbstractDoubleMatrix getParticles()
	{
		return state;
	}

	
	public static ModelFilter getModelFilter()
	{
		return new PFModelFilter();
	}
}


class PFModelFilter implements ModelFilter
{
	public boolean canUseDynamicModel(DynamicModel dynModel)
	{
		return (dynModel instanceof ParticleCloudPredictorFunction);
	}


	public boolean canUseSensorModel(SensorModel sensModel)
	{
		return (sensModel instanceof PosteriorProbabilityDensityFunction);
	}
}

