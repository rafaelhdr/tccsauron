package br.com.r4j.robosim;

import java.util.List;
import br.com.r4j.robosim.estimator.*;


public interface SensorModelsManager
{
	public DynamicModel getDynamicModel();

	
	public List getSensors();


	public List getSensorModels();
}
