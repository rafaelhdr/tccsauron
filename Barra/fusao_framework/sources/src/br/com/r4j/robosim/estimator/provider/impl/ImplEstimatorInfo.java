package br.com.r4j.robosim.estimator.provider.impl;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.impl.BaseEstimator;
import br.com.r4j.robosim.estimator.provider.EstimatorInfo;


public class ImplEstimatorInfo extends EstimatorInfo
{
	private static Log log = LogFactory.getLog(ImplEstimatorInfo.class.getName());

	protected Class est = null;
	protected String strConfBase = null;
	protected PropertiesHolder props = null;


	public ImplEstimatorInfo(String name)
	{
		super(name);
	}


	public void setEstimatorClass(Class est, String strConfBase, PropertiesHolder props)
	{
		this.est = est;
		this.strConfBase = strConfBase;
		this.props = props;
	}
	
	
	public Class getEstimatorClass()	{return this.est;}
	public Estimator createEstimator()
	{
		try
		{
			Estimator estInst = (Estimator) est.newInstance();
			if (estInst instanceof BaseEstimator)
			{
				log.debug("this.getName(): " + this.getName());
				((BaseEstimator) estInst).setInstanceName(this.getName());
			}
			if (estInst instanceof Configurable)
				((Configurable) estInst).configure(props, strConfBase);
			return estInst;
		}
		catch (Exception e)
		{
			return null;
		}
	}


	public EstimatorInfo getCopy()
	{
		ImplEstimatorInfo snsModelCopy = new ImplEstimatorInfo(name);
		 
		snsModelCopy.invModel = invModel;
		snsModelCopy.minSensor = maxSensor;
		snsModelCopy.maxSensor = maxSensor;
		snsModelCopy.dynModelNecessity = dynModelNecessity;

		snsModelCopy.est = est;
		snsModelCopy.strConfBase = strConfBase;
		snsModelCopy.props = props;

		return snsModelCopy;
	}
}
