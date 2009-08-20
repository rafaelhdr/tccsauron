package br.com.r4j.robosim.estimator.provider.impl;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.provider.DynamicModelInfo;


public class ImplDynamicModelInfo extends DynamicModelInfo
{
	private static Log log = LogFactory.getLog(ImplSensorModelInfo.class.getName());

	protected DynamicModel est = null;

	protected Class estCls = null;
	protected String strConfBase = null;
	protected PropertiesHolder props = null;


	public ImplDynamicModelInfo(String name)
	{
		super(name);
	}


	public void setDynamicModel(Class  est, String strConfBase, PropertiesHolder props)
	{
		this.estCls = est; 
		this.strConfBase = strConfBase;
		this.props = props;
	}

	public DynamicModel getDynamicModel()	
	{
		if (est == null)
		{		
			try
			{
				est = (DynamicModel) estCls.newInstance();
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


	public DynamicModelInfo getCopy()
	{
		ImplDynamicModelInfo snsModelCopy = new ImplDynamicModelInfo(name);
		 
		snsModelCopy.sens = sens;

		snsModelCopy.estCls = estCls;
		snsModelCopy.strConfBase = strConfBase;
		snsModelCopy.props = props;
	
		snsModelCopy.est = null;

		return snsModelCopy;
	}
}
