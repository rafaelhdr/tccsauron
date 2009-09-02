package br.com.r4j.robosim.estimator.provider;


public class SensorModelInfo
{
	protected SensorInfo sens = null;
	protected String name = null;
	protected boolean bUsesInvertedModel = false;


	public SensorModelInfo(String name)
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


	public void setUsesInvertedModel(boolean b)	{this.bUsesInvertedModel = b;}
	public boolean usesInvertedModel()	{return this.bUsesInvertedModel;}
	public boolean supportInvertedModel()	{return this.bUsesInvertedModel;}


	public SensorModelInfo getCopy()
	{
		SensorModelInfo snsModelCopy = new SensorModelInfo(name);
		 
		snsModelCopy.sens = sens;
		snsModelCopy.bUsesInvertedModel = bUsesInvertedModel;

		return snsModelCopy;
	}
}
