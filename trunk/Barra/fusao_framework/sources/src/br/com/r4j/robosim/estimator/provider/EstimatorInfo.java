package br.com.r4j.robosim.estimator.provider;



public abstract class EstimatorInfo
{
	protected String name = null;
	protected boolean invModel = false;
	protected int minSensor = -1;
	protected int maxSensor = -1;
	protected int dynModelNecessity = -1;


	public EstimatorInfo(String name)
	{
		this.name = name;
	}


	public void setUsesInvertedModel(boolean invModel)	{this.invModel = invModel;}
	public boolean usesInvertedModel()	{return this.invModel;}

	public void setMinSensors(int minSensor)	{this.minSensor = minSensor;}
	public int getMinSensors()	{return this.minSensor;}

	public void setMaxSensors(int maxSensor)	{this.maxSensor = maxSensor;}
	public int getMaxSensors()	{return this.maxSensor;}

	public void setDynamicModeNecessity(int dynModelNecessity) {this.dynModelNecessity = dynModelNecessity;}
	public int getDynamicModeNecessity() {return this.dynModelNecessity;}

	
	public String getName()
	{
		return name;
	}

	
	public String toString()
	{
		return name;
	}


	public abstract Class getEstimatorClass();


	public abstract EstimatorInfo getCopy();
}
