package br.com.r4j.robosim.estimator.provider.impl;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.provider.SensorModelInfo;


public class ImplSensorModelInfo extends SensorModelInfo
{
	private static Log log = LogFactory.getLog(ImplSensorModelInfo.class.getName());

	protected Class estCls = null;
	protected String strConfBase = null;
	protected PropertiesHolder props = null;
	
	protected SensorModel est = null;


	public ImplSensorModelInfo(String name)
	{
		super(name);
	}


	public void setSensorModel(Class  est, String strConfBase, PropertiesHolder props)
	{
		this.estCls = est; 
		this.strConfBase = strConfBase;
		this.props = props;
	}
	
	
	public SensorModel getSensorModel()
	{
		if (true || est == null)
		{		
			if (estCls == null)
				return null;
			try
			{
				est = (SensorModel) estCls.newInstance();
				if (est instanceof Configurable)
					((Configurable) est).configure(props, strConfBase);
			}
			catch (InstantiationException e)
			{
				log.error("erro", e);
				return null;
			}
			catch (IllegalAccessException e)
			{
				log.error("erro", e);
				return null;
			}
		}
		return est;
	}
	
	
	public SensorModelInfo getCopy()
	{
		ImplSensorModelInfo snsModelCopy = new ImplSensorModelInfo(name);
		 
		snsModelCopy.sens = sens;
		snsModelCopy.bUsesInvertedModel = bUsesInvertedModel;

		snsModelCopy.estCls = estCls;
		snsModelCopy.strConfBase = strConfBase;
		snsModelCopy.props = props;
	
		snsModelCopy.est = null;

		return snsModelCopy;
	}
}
