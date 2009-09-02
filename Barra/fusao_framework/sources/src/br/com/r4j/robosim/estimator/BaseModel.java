package br.com.r4j.robosim.estimator;


public interface BaseModel
{
	public String getName();


	public void setSensor(Sensor sns);


	public int getDataDimension();
}
