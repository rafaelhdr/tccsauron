package br.com.r4j.robosim.estimator.provider.impl;

import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.provider.SensorInfo;


/** @modelguid {961B2FD4-00DE-45E6-9D9C-BB9E915DB729} */
public class ImplSensorInfo extends SensorInfo
{
	/** @modelguid {67EB85F0-0E52-4864-95B4-A9C30EE4972C} */
	private Sensor est = null;


	/** @modelguid {796EC6DD-1E60-47CA-BD0C-B29A7CFD76F0} */
	public ImplSensorInfo(String name)
	{
		super(name);
	}


	/** @modelguid {89C3159A-669C-4AC4-8E87-28096830B276} */
	public void setSensor(Sensor est)	{this.est = est;}
	/** @modelguid {580F8219-5AF3-4255-B8D4-167EEB16CE3E} */
	public Sensor getSensor()	{return this.est;}
}
