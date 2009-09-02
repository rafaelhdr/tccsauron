package br.com.r4j.robosim.estimator;

import java.util.List;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.robosim.EstimatorRenderer;


public interface Estimator
{
	public List getRenderers();

	public void addSensorModel(SensorModel sensModel, Sensor sens) throws ConfiguratorException;

	public List getSensorModels();

	public void setDynamicModel(DynamicModel dynModel, Sensor dynModelSensor) throws ConfiguratorException;

	public DynamicModel getDynamicModel();

	public void estimate(boolean bUpdateRenderers);

	public String getName();

	public String getInstanceName();

	public void setInstanceName(String a);

	public AbstractDoubleVector getMean();

	public AbstractDoubleSquareMatrix getCovariance();

	public void setState(AbstractDoubleVector stateEstimate, AbstractDoubleSquareMatrix stateCovarEstimate);

	public void addRobotTracker(EstimatorRenderer track);
}
