package br.com.r4j.robosim.estimator.provider.impl;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.provider.DynamicModelInfo;


/** @modelguid {275A98E4-D324-48CE-BD08-611287971967} */
public class ImplDynamicModelInfo extends DynamicModelInfo
{
	/** @modelguid {1E74709F-8A4E-4DDA-B48E-823272FBE893} */
	private static Log log = LogFactory.getLog(ImplSensorModelInfo.class.getName());

	/** @modelguid {0B89CF4E-E359-43F3-80EE-EE8C76B5753A} */
	protected DynamicModel est = null;

	/** @modelguid {D1A64C29-65D0-45BE-9E41-BCE5C1287B2C} */
	protected Class estCls = null;
	/** @modelguid {A2679945-09D4-47E6-8D00-75D93365CAEF} */
	protected String strConfBase = null;
	/** @modelguid {EAC18763-C02F-4365-908F-CAA6E59EDE89} */
	protected PropertiesHolder props = null;


	/** @modelguid {B3B032AB-0954-45A7-B684-B883E8639F6E} */
	public ImplDynamicModelInfo(String name)
	{
		super(name);
	}


	/** @modelguid {8109A3A3-6921-42F7-9F6C-4058DECB4DAC} */
	public void setDynamicModel(Class  est, String strConfBase, PropertiesHolder props)
	{
		this.estCls = est; 
		this.strConfBase = strConfBase;
		this.props = props;
	}

	/** @modelguid {7CE3AB95-7A0A-49D9-B44D-0193B8DE61EA} */
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


	/** @modelguid {89A2F2CC-62E4-419D-B968-E7CDBCA41F5D} */
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
