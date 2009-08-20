package br.com.r4j.robosim.estimator.provider.impl;

import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.provider.SensorInfo;


public class ImplSensorInfo extends SensorInfo
{
	private Sensor est = null;


	public ImplSensorInfo(String name)
	{
		super(name);
	}


	public void setSensor(Sensor est)	{this.est = est;}
	public Sensor getSensor()	{return this.est;}
}
