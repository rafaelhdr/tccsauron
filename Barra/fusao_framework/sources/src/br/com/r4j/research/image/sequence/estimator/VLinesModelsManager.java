package br.com.r4j.research.image.sequence.estimator;

import java.util.Date;
import java.util.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.math.MathConsts;

import br.com.r4j.commons.util.*;
import br.com.r4j.commons.util.Collections;
import br.com.r4j.configurator.Configurable;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.research.image.sequence.featurematch.vline.*;
import br.com.r4j.research.image.sequence.*;
import br.com.r4j.research.vline.*;
import br.com.r4j.robosim.model.RadarModel;
import br.com.r4j.robosim.estimator.*;
import br.com.r4j.robosim.*;


public class VLinesModelsManager implements Configurable, MapDependent, SensorModelsManager
{
	private static Log log = LogFactory.getLog(RadarModel.class.getName());
	private static Log logSens = LogFactory.getLog("vline");

	private WorldMap map = null;
	private CameraModel camModel = null;
	private VLineMap lineMap = null;

	private Random rnd = null;

	private VLinesMapSensorModel snsMapModel = null;
	private VLinesStateSensorModel snsStateModel = null;
	private VLinesDynModel dynModel = null;
	private VLineStateSensor snsState = null;
	private VLineMapSensor snsMap = null;

	private DifferenceSimpleMatchingPhaseMatcher matchingPhase = null;


	public VLinesModelsManager()
	{
		this(new VLineStateSensor(), new VLineMapSensor());
	}


	public VLinesModelsManager(VLineStateSensor snsState, VLineMapSensor snsMap)
	{
		rnd = new Random((new Date()).getTime());

		lineMap = new VLineMap();
		camModel = new CameraModel();

		matchingPhase = new DifferenceSimpleMatchingPhaseMatcher();
		snsMapModel = new VLinesMapSensorModel(matchingPhase);
		snsStateModel = new VLinesStateSensorModel(matchingPhase);
		dynModel = new VLinesDynModel();

		this.snsState = snsState;
		this.snsMap = snsMap;

		snsState.setMapSensor(snsMap);

		snsMapModel.setDynModel(dynModel);
		snsStateModel.setDynModel(dynModel);

		snsStateModel.setVLinesMapSensorModel(snsMapModel);

		snsMapModel.setCameraModel(camModel);
		snsStateModel.setCameraModel(camModel);
		snsMap.setCameraModel(camModel);
		snsState.setCameraModel(camModel);

		snsMapModel.setVLineMap(lineMap);
		snsStateModel.setVLineMap(lineMap);
		dynModel.setVLineMap(lineMap);

		dynModel.setMapSensorModel(snsMapModel);
	}


	public void setWorldMap(WorldMap map)
	{
		this.map = map;
		dynModel.setWorldMap(map);
		snsStateModel.setWorldMap(map);
		snsMapModel.setWorldMap(map);
	}


	public void configure(PropertiesHolder props, String strBaseKey)
	{
		log.debug("z-rot-graus: " + props.getDoubleProperty(strBaseKey + "/z-rot-graus").doubleValue());
		camModel.setZRotCamera(props.getDoubleProperty(strBaseKey + "/z-rot-graus").doubleValue()*MathConsts.PI/180);
		camModel.setXOffset(props.getDoubleProperty(strBaseKey + "/x-offset").doubleValue());
		camModel.setYOffset(props.getDoubleProperty(strBaseKey + "/y-offset").doubleValue());
		camModel.setFocusDistance((int) props.getDoubleProperty(strBaseKey + "/focal-distance").doubleValue());
		camModel.setUAxisPixelCenter((int) props.getDoubleProperty(strBaseKey + "/u-center").doubleValue());
		camModel.setVAxisPixelCenter((int) props.getDoubleProperty(strBaseKey + "/v-center").doubleValue());
		camModel.setUAxisPixelCount((int) props.getDoubleProperty(strBaseKey + "/u-count").doubleValue());
		camModel.setVAxisPixelCount((int) props.getDoubleProperty(strBaseKey + "/v-count").doubleValue());

		matchingPhase.setUseWindow(props.getBooleanProperty(strBaseKey + "/use-window").booleanValue());
		matchingPhase.setUseDirectionWindow(props.getBooleanProperty(strBaseKey + "/use-direct-window").booleanValue());
		matchingPhase.setDirectWindowSize(props.getIntegerProperty(strBaseKey + "/direct-window-size").intValue());
		matchingPhase.setMinDepth(props.getIntegerProperty(strBaseKey + "/min-depth").intValue());
		matchingPhase.setHashVectorTreshold(props.getDoubleProperty(strBaseKey + "/hash-treshold").doubleValue());
		matchingPhase.setSimpleMatch(props.getBooleanProperty(strBaseKey + "/simple-match").booleanValue());

		VLinePerfil.MAX_ATTENUATION = (props.getDoubleProperty(strBaseKey + "/hash-attenuation").doubleValue());
		VLinePerfil.PERFIL_HISTORICO = (props.getIntegerProperty(strBaseKey + "/hash-hist-size").intValue());
		VLinePerfil.PERFIL_SIZE = 2 * (props.getIntegerProperty(strBaseKey + "/hash-side-band").intValue()) + 1;
		VLinePerfil.APPLY_HISTOGRAM = (props.getBooleanProperty(strBaseKey + "/compensate-illumination").booleanValue());
	}


//	public VLinesDynModel getDynamicModel()
	public DynamicModel getDynamicModel()
	{
		return dynModel;
	}

	
	public List getSensors()
	{
		return Collections.createList(snsMap, snsState);
	}


	public List getSensorModels()
	{
		return Collections.createList(snsMapModel, snsStateModel);
	}


	public VLinesStateSensorModel getSensorModelState()
	{
		return snsStateModel;
	}


	public VLinesMapSensorModel getSensorModelMap()
	{
		return snsMapModel;
	}


	public VLineMapSensor getSensorMap()
	{
		return snsMap;
	}


	public VLineStateSensor getSensorState()
	{
		return snsState;
	}
}


