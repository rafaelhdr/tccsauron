package br.com.r4j.robosim.estimator.provider;


public class DynamicModelInfo
{
	protected SensorInfo sens = null;
	protected String name = null;


	public DynamicModelInfo(String name)
	{
		this.name = name;
	}


	public void setSensorInfo(SensorInfo sens)
	{
		this.sens = sens;
	}

	
	public SensorInfo getSensorInfo()
	{
		return sens;
	}


	public String getName()
	{
		return name;
	}

	
	public String toString()
	{
		return name;
	}


	public DynamicModelInfo getCopy()
	{
		DynamicModelInfo snsModelCopy = new DynamicModelInfo(name);
		 
		snsModelCopy.sens = sens;

		return snsModelCopy;
	}
}
