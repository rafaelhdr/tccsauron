package br.com.r4j.robosim.estimator.provider;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.robosim.ActionFileRobotPlayer;
import br.com.r4j.robosim.InteractiveRobotPlayer;
import br.com.r4j.robosim.RealRobotPlayer;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.provider.impl.ImplDynamicModelInfo;
import br.com.r4j.robosim.estimator.provider.impl.ImplEstimatorInfo;
import br.com.r4j.robosim.estimator.provider.impl.ImplSensorInfo;
import br.com.r4j.robosim.estimator.provider.impl.ImplSensorModelInfo;


/**
 *
 *
 *
 *

<estimators>
	<estimator_id  class_name="la.la.la" name="lu lu lu" id="qww" inverse_model="ss" min_sensors="1" max_sensors="-1" dyn_model="required/opt/dont_use"/>
</estimators>

<real_world>
	<sensors>
		<sensor_id class="la.la.la" name="lu lu lu" is_movement_sensor=""/>
	</sensors>
	<sensor_models>
		<sensor_model_id class="la.la.la" name="lu lu lu" sensor="qww"/>
	</sensor_models>
	<dynamic_models>
		<dynamic_model_1 class="la.la.la" name="lu lu lu" sensor="qww"/>
	</dynamic_models>
</real_world>

<sim_world>
	<sensors>
		<sensor class="la.la.la" name="lu lu lu" id="qww"/>
	</sensors>
	<sensor_models>
		<sensor_model class="la.la.la" name="lu lu lu" sensor="qww" inverse_model_avail="qq"/>
	</sensor_models>
</sim_world>


 *
 *
 *
 *
 *
 */
public class EstimationObjectsProvider
{
	private static Log log = LogFactory.getLog(EstimationObjectsProvider.class.getName());

	private static final int REQUIRED = 1;
	private static final int OPTIONAL = 2;
	private static final int DONT_USE = 3;
	private static final String REQUIRED_STR = "REQUIRED";
	private static final String OPTIONAL_STR = "OPTIONAL";
	private static final String DONT_USE_STR = "DONT_USE";

	private ArrayList listEstimatorsInfo = null;

	private ArrayList listRealSensorsInfo = null;
	private ArrayList listRealSensorModelsInfo = null;
	private HashMap mapRealSensorId2SensorInfo = null;
	private ArrayList listRealDynModelInfo = null;

	private ArrayList listSimSensorsInfo = null;
	private ArrayList listSimSensorModelsInfo = null;
	private HashMap mapSimSensorId2SensorInfo = null;
	private ArrayList listSimDynModelInfo = null;

	private InteractiveRobotPlayer intPlayer = null;
	private RealRobotPlayer realPlayer = null;
	private ActionFileRobotPlayer actionFilePlayer = null;



	public EstimationObjectsProvider()
	{
		listEstimatorsInfo = new ArrayList();

		listRealSensorsInfo = new ArrayList();
		listRealSensorModelsInfo = new ArrayList();
		mapRealSensorId2SensorInfo = new HashMap();

		listSimSensorsInfo = new ArrayList();
		listSimSensorModelsInfo = new ArrayList();
		mapSimSensorId2SensorInfo = new HashMap();
		
		listSimDynModelInfo = new ArrayList();
		listRealDynModelInfo = new ArrayList();

		intPlayer = new InteractiveRobotPlayer();
		realPlayer = new RealRobotPlayer();
		actionFilePlayer = new ActionFileRobotPlayer();
	}


	public void loadConfiguration(PropertiesHolder props, String strPathBase) throws ConfiguratorException
	{
		listEstimatorsInfo.clear();
		listRealSensorsInfo.clear();
		listRealSensorModelsInfo.clear();
		mapRealSensorId2SensorInfo.clear();
		listSimSensorsInfo.clear();
		listSimSensorModelsInfo.clear();
		mapSimSensorId2SensorInfo.clear();
		listSimDynModelInfo.clear();
		listRealDynModelInfo.clear();

		Iterator itProps = props.getSubGroupOfKeys(strPathBase + "estimators", 1).iterator();
		while (itProps.hasNext())
		{
			String strProp = (String) itProps.next();
			log.debug("estimador: " + strProp);
//			Estimator est = (Estimator) props.getObjectProperty(strProp);
			ImplEstimatorInfo estInfo = new ImplEstimatorInfo(props.getStringProperty(strProp + "/name"));
			listEstimatorsInfo.add(estInfo);
			if (props.containsProperty(strProp + "/inverse_model"))
				estInfo.setUsesInvertedModel(props.getBooleanProperty(strProp + "/inverse_model").booleanValue());
			else
				estInfo.setUsesInvertedModel(false);

			if (!props.containsProperty(strProp + "/class_name"))
				throw new ConfiguratorException(EstimationObjectsProvider.class , "propriedade class_name é ogrigatória: " + estInfo.getName());
			Class clsEstimator = null;
			try
			{
				clsEstimator = Class.forName(props.getStringProperty(strProp + "/class_name"));
			}
			catch (Exception e)
			{
				log.error("erro", e);
				throw new ConfiguratorException(EstimationObjectsProvider.class , "class não existe: " + props.getStringProperty(strProp + "class_name"));
			}
			estInfo.setEstimatorClass(clsEstimator, strProp, props);

			if (!props.containsProperty(strProp + "/min_sensors"))
				throw new ConfiguratorException(EstimationObjectsProvider.class , "propriedade min_sensors é ogrigatória: " + estInfo.getName());
			estInfo.setMinSensors(props.getIntegerProperty(strProp + "/min_sensors").intValue());

			if (!props.containsProperty(strProp + "/max_sensors"))
				throw new ConfiguratorException(EstimationObjectsProvider.class , "propriedade max_sensors é ogrigatória: " + estInfo.getName());
			estInfo.setMaxSensors(props.getIntegerProperty(strProp + "/max_sensors").intValue());

			if (props.containsProperty(strProp + "/dyn_model"))
			{
				String strDynModelNecessity = props.getStringProperty(strProp + "/dyn_model").toUpperCase();
				if (strDynModelNecessity.equals(OPTIONAL_STR))
					estInfo.setDynamicModeNecessity(OPTIONAL);
				else if (strDynModelNecessity.equals(DONT_USE_STR))
					estInfo.setDynamicModeNecessity(DONT_USE);
				else
					estInfo.setDynamicModeNecessity(REQUIRED);
			}
			else
				estInfo.setDynamicModeNecessity(REQUIRED);

//			estInfo.setEstimator(est);
		}

		this.loadSensors(props, props.getSubGroupOfKeys(strPathBase + "real_world/sensors", 1), props.getSubGroupOfKeys(strPathBase + "real_world/sensor_models", 1), props.getSubGroupOfKeys(strPathBase + "real_world/dynamic_models", 1), listRealSensorsInfo, listRealSensorModelsInfo, listRealDynModelInfo, mapRealSensorId2SensorInfo);
		this.loadSensors(props, props.getSubGroupOfKeys(strPathBase + "sim_world/sensors", 1), props.getSubGroupOfKeys(strPathBase + "sim_world/sensor_models", 1), props.getSubGroupOfKeys(strPathBase + "sim_world/dynamic_models", 1), listSimSensorsInfo, listSimSensorModelsInfo, listSimDynModelInfo, mapSimSensorId2SensorInfo);
	}


	private void loadSensors(PropertiesHolder props, List listProps, List listPropsModels, List listPropsDynModels, ArrayList listSensorsInfo, ArrayList listSensorModelsInfo, ArrayList listDynModelInfo, HashMap mapSensorId2SensorInfo) throws ConfiguratorException
	{
		Iterator itProps = listProps.iterator();
		while (itProps.hasNext())
		{
			String strProp = (String) itProps.next();
			log.debug("sensor: " + strProp);
			Sensor sens = (Sensor) props.getObjectProperty(strProp);
			if (sens instanceof Configurable)
				((Configurable) sens).configure(props, strProp);
			ImplSensorInfo sensInfo = new ImplSensorInfo(props.getStringProperty(strProp + "/name"));

			String strId = props.getStringProperty(strProp + "/id");
//			String strId = strProp.substring(0, strProp.length() - 1);
//			strId = strId.substring(strId.lastIndexOf("/") + 1, strId.length());

			sensInfo.setSensor(sens);
			listSensorsInfo.add(sensInfo);
			mapSensorId2SensorInfo.put(strId, sensInfo);
		}

		itProps = listPropsModels.iterator();
		while (itProps.hasNext())
		{
			String strProp = (String) itProps.next();
			log.debug("sensor model: " + strProp);
			SensorModel sensModel = (SensorModel) props.getObjectProperty(strProp);
			ImplSensorModelInfo sensModelInfo = new ImplSensorModelInfo(props.getStringProperty(strProp + "/name"));
			if (props.containsProperty(strProp + "/inverse_model_avail"))
				sensModelInfo.setUsesInvertedModel(props.getBooleanProperty(strProp + "/inverse_model_avail").booleanValue());
			else
				sensModelInfo.setUsesInvertedModel(false);

			if (!props.containsProperty(strProp + "/sensor"))
				throw new ConfiguratorException(EstimationObjectsProvider.class , "propriedade sensor é ogrigatória: " + sensModelInfo.getName());
			sensModelInfo.setSensorInfo((SensorInfo) mapSensorId2SensorInfo.get(props.getStringProperty(strProp + "/sensor")));

			sensModelInfo.setSensorModel(sensModel.getClass(), strProp, props);
			listSensorModelsInfo.add(sensModelInfo);
		}

		itProps = listPropsDynModels.iterator();
		while (itProps.hasNext())
		{
			String strProp = (String) itProps.next();
			log.debug("dynamic model: " + strProp);
			DynamicModel dynModel = (DynamicModel) props.getObjectProperty(strProp);
			ImplDynamicModelInfo dynModelInfo = new ImplDynamicModelInfo(props.getStringProperty(strProp + "/name"));

			if (!props.containsProperty(strProp + "/sensor"))
				throw new ConfiguratorException(EstimationObjectsProvider.class , "propriedade sensor é ogrigatória: " + dynModelInfo.getName());
			dynModelInfo.setSensorInfo((SensorInfo) mapSensorId2SensorInfo.get(props.getStringProperty(strProp + "/sensor")));

			dynModelInfo.setDynamicModel(dynModel.getClass(), strProp, props);
			listDynModelInfo.add(dynModelInfo);
		}
	}


	public List getEstimatorsInfo()
	{
		return listEstimatorsInfo;
	}


	public List getSensorModelsInfo(boolean bSimulationEnv)
	{
		if (bSimulationEnv)
			return listSimSensorModelsInfo;
		else
			return listRealSensorModelsInfo;
	}


	public DynamicModelInfo getDynamicModelInfo(boolean bSimulationEnv)
	{
		if (bSimulationEnv)
			return (DynamicModelInfo) listSimDynModelInfo.get(0);
		else
			return (DynamicModelInfo) listRealDynModelInfo.get(0);
	}


	public InteractiveRobotPlayer getSimulatedInteractivePlayer()
	{
		return intPlayer;
	}


	public ActionFileRobotPlayer getSimulatedActionFilePlayer()
	{
		return actionFilePlayer;
	}


	public RealRobotPlayer getRealPlayer()
	{
		return realPlayer;
	}


	public Sensor getSensor(SensorInfo sensInfo)
	{
		return ((ImplSensorInfo) sensInfo).getSensor();
	}


	public SensorModel getSensorModel(SensorModelInfo sensInfo)
	{
		return ((ImplSensorModelInfo) sensInfo).getSensorModel();
	}


	public DynamicModel getDynamicModel(DynamicModelInfo sensInfo)
	{
		return ((ImplDynamicModelInfo) sensInfo).getDynamicModel();
	}


	public Estimator createEstimator(EstimationComponentsInfo estCompsInfo) throws ConfiguratorException
	{
		Estimator est = ((ImplEstimatorInfo) estCompsInfo.getEstimator()).createEstimator();
		Iterator itSensModels = estCompsInfo.getSensorModels().iterator();
		while (itSensModels.hasNext())
		{
			ImplSensorModelInfo sensModel = (ImplSensorModelInfo) itSensModels.next();
			SensorModel snsMdl = sensModel.getSensorModel();
			Sensor sns = ((ImplSensorInfo) sensModel.getSensorInfo()).getSensor();
			snsMdl.setSensor(sns);
			est.addSensorModel(snsMdl, sns);
		}

		if (estCompsInfo.getEstimator().getDynamicModeNecessity() == REQUIRED || 
			(estCompsInfo.getEstimator().getDynamicModeNecessity() == OPTIONAL))
		{
			ImplDynamicModelInfo dynModel = (ImplDynamicModelInfo) estCompsInfo.getDynamicModel();
			DynamicModel dynMdl = dynModel.getDynamicModel();
			Sensor sns = ((ImplSensorInfo) dynModel.getSensorInfo()).getSensor();
			dynMdl.setSensor(sns);
			est.setDynamicModel(dynMdl, sns);
		}
		return est;
	}
}
