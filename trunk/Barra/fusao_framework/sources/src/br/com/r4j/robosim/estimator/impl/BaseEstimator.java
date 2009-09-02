package br.com.r4j.robosim.estimator.impl;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.EstimateObserver;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.PredictedEstimateConsumer;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;


public abstract class BaseEstimator implements Estimator
{
	private static Log log = LogFactory.getLog(BaseEstimator.class.getName());
	private static Log logTime = LogFactory.getLog("time");

	protected AbstractDoubleVector stateEstimatePred = null;
	protected AbstractDoubleSquareMatrix stateCovarEstimatePred = null;
	
	protected AbstractDoubleVector stateEstimate = null;
	protected AbstractDoubleSquareMatrix stateCovarEstimate = null;


	private ArrayList listTrackers = null;
	private ArrayList listEstimateObservers = null;
	protected boolean bAltered = false;
	protected ArrayList listSensModels = null;
	protected ArrayList listSens = null;
	protected ArrayList listPredEstimateConsumer = null;

	protected DynamicModel dynModel = null;
	protected Sensor dynModelSensor = null;
	
	private int iterationCount = 0;


	public BaseEstimator()
	{
		bAltered = true;
		listSensModels = new ArrayList();
		listSens = new ArrayList();
		listTrackers = new ArrayList();
		listEstimateObservers = new ArrayList();
		listPredEstimateConsumer = new ArrayList();
		
		iterationCount = 0;
	}

	public abstract String getName();

	private String strInstName = null;
	public void setInstanceName(String a)
	{
		this.strInstName = a;
	}
	
	public String getInstanceName()
	{
		if (strInstName != null)
			return strInstName;
		else
			return this.getName();
	}


	public List getRenderers()
	{
		return new ArrayList();
	}


	public void estimate(boolean bUpdateRenderers)
	{
		iterationCount++;
		if (bAltered)
		{
			bAltered = false;
			this.prepareBuffers();
		}
		long time = System.currentTimeMillis();
		this.doEstimate();
		if (logTime.isDebugEnabled())
			logTime.debug(this.getClass().getName() + ": " + (System.currentTimeMillis() - time));
		if (bUpdateRenderers)
		{
			this.fireNewRobotPose();
			this.fireNewEstimate();
		}
	}


	protected abstract void doEstimate();


	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException
	{
		bAltered = true;
		listSensModels.add(sensModel);
		listSens.add(sens);
		if (sensModel instanceof PredictedEstimateConsumer)
			listPredEstimateConsumer.add(sensModel);
	}

	
	protected int getIerationCount()
	{
		return iterationCount;
	}


	public List getSensorModels()
	{
		return listSensModels;
	}


	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException
	{
		bAltered = true;
		this.dynModel = dynModel;
		this.dynModelSensor = dynModelSensor;
	}


	public DynamicModel getDynamicModel()
	{
		return dynModel;
	}

	
	protected void prepareBuffers()
	{
	}


	public void setPredictedState(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		this.stateEstimatePred = stateEstimate;
		this.stateCovarEstimatePred = stateCovarEstimate;

		this.stateEstimate = stateEstimate;
		this.stateCovarEstimate = stateCovarEstimate;
		
		Iterator itPredEstimateConsumer = listPredEstimateConsumer.iterator();
		while (itPredEstimateConsumer.hasNext())
		{
			PredictedEstimateConsumer estConsumer = (PredictedEstimateConsumer) itPredEstimateConsumer.next();
			estConsumer.predictedEstimateAvailable(stateEstimate, stateCovarEstimate); 
		}
	}


	public void setState(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		this.stateEstimate = stateEstimate;
		this.stateCovarEstimate = stateCovarEstimate;
	}


	public void setCovariance(AbstractDoubleSquareMatrix stateCovarEstimate)
	{
		this.stateCovarEstimate = stateCovarEstimate;
	}


	public AbstractDoubleVector getPredictedMean()
	{
		return stateEstimatePred;
	}


	public AbstractDoubleSquareMatrix getPredictedCovariance()
	{
		return stateCovarEstimatePred;
	}


	public AbstractDoubleVector getMean()
	{
		return stateEstimate;
	}


	public AbstractDoubleSquareMatrix getCovariance()
	{
		return stateCovarEstimate;
	}


	public void addRobotTracker(EstimatorRenderer track)
	{
		listTrackers.add(track);
	}
	

	public void addEstimateObserver(EstimateObserver obs)
	{
		listEstimateObservers.add(obs);
	}


	protected void fireNewRobotPose()
	{
		Pose2D pose = new Pose2D(this.getMean());
		Iterator itTrackers = listTrackers.iterator();
		while (itTrackers.hasNext())
		{
			EstimatorRenderer track = (EstimatorRenderer) itTrackers.next();
			track.newPose(pose);
		}
	}


	private void fireNewEstimate()
	{
		Iterator itObs = listEstimateObservers.iterator();
		while (itObs.hasNext())
		{
			EstimateObserver obs = (EstimateObserver) itObs.next();
			obs.newEstimate(this.getMean(), this.getCovariance());
		}
	}
}
// 114