package br.com.r4j.research.image.sequence.featurematch.vline.test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.awt.geom.*;
import java.awt.image.BufferedImage;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.configurator.WebStartConfigurator;
import br.com.r4j.robosim.estimator.DynamicModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.impl.*;

import br.com.r4j.robosim.realrobot.*;
import br.com.r4j.robosim.*;

import br.com.r4j.jmr.datasource.*;
import br.com.r4j.research.image.sequence.*;
import br.com.r4j.research.image.sequence.estimator.*;
import br.com.r4j.research.vline.*;

import br.com.r4j.commons.util.*;
import br.com.r4j.image.operation.threebandpacked.*;


public class EstimatorTest
{
	private static Log log = LogFactory.getLog(EstimatorTest.class.getName());
	private static Log logOutput = LogFactory.getLog("output");

	
	private static RealRobotPlayer realPlayer = null;
	private static OdoSensor odoSensor = null;
	private static WorldMap worldMap = null;
	private static ArrayList listEstimators = null;

	
	public static void main(String [] args)
	{
		try
		{
			WebStartConfigurator.createConfigurator("conf/conf.xml");
			Configurator conf = Configurator.getInstance();
			PropertiesHolder props = conf.getPropertiesHolder();

			worldMap = new WorldMap();
			worldMap.setMapFile(props.getFileProperty("/robosim/worldmap"));

			realPlayer = new RealRobotPlayer();

			realPlayer.setWorldMap(worldMap);
			realPlayer.setReadingsFile(props.getFileProperty("/robosim/readings_vline"),
									null);
//									props.getFileProperty("/robosim/realpose"));

			odoSensor = new OdoSensor();
			odoSensor.configure(props, "/odosensor");
			realPlayer.addSensor(odoSensor);

			Estimator est = null;
			listEstimators = new ArrayList();

			VLinesModelsManager manMaster = new VLinesModelsManager();


			est = new AditiveNosieExtendedKalmanFilter();
			listEstimators.add(est);
			prepareEstimator(est, manMaster);

			est = new AditiveNosieUnscentedKalmanFilter();
			listEstimators.add(est);
			prepareEstimatorSlave(est, new VLinesModelsManager(manMaster.getSensorState(), manMaster.getSensorMap()));

			est = new AditiveNosieEigenvector2SigmaPointsUnscentedKalmanFilter();
			listEstimators.add(est);
			prepareEstimatorSlave(est, new VLinesModelsManager(manMaster.getSensorState(), manMaster.getSensorMap()));

			est = new AditiveNosieSquareRootUnscentedKalmanFilter();
			listEstimators.add(est);
			prepareEstimatorSlave(est, new VLinesModelsManager(manMaster.getSensorState(), manMaster.getSensorMap()));
/*
//*/
/*
//*/
			realPlayer.setInitialState(worldMap.getInitialLocalization(),
										worldMap.getInitialLocalizationCovar());
//						new DoubleSquareMatrix(new double [][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 0.001}}));

			execute();
		}
		catch (ConfiguratorException e)
		{
			e.printStackTrace();
			return;
		}
		catch (Exception e)
		{
			e.printStackTrace();
			return;
		}
	}


	public EstimatorTest()
	{
	}


	private static void prepareEstimator(Estimator est, VLinesModelsManager vlineManager) throws ConfiguratorException
	{
		Configurator conf = Configurator.getInstance();
		PropertiesHolder props = conf.getPropertiesHolder();

		vlineManager.setWorldMap(worldMap);
		vlineManager.getSensorModelMap().configure(props, "/vlinesnsmodel");
		vlineManager.getSensorModelState().configure(props, "/vlinesnsmodel");
		((VLinesDynModel) vlineManager.getDynamicModel()).configure(props, "/vlinedynmodel");
		vlineManager.configure(props, "/vlinemanager");

		vlineManager.getDynamicModel().setSensor(odoSensor);
		vlineManager.getSensorModelMap().setSensor(vlineManager.getSensorMap());
		vlineManager.getSensorModelState().setSensor(vlineManager.getSensorState());

		est.setDynamicModel(vlineManager.getDynamicModel(), odoSensor);
		est.addSensorModel(vlineManager.getSensorModelMap(), vlineManager.getSensorMap());
		est.addSensorModel(vlineManager.getSensorModelState(), vlineManager.getSensorState());

		realPlayer.addSensor(vlineManager.getSensorMap());
		realPlayer.addSensor(vlineManager.getSensorState());
		realPlayer.addSensorModel(vlineManager.getSensorModelMap());
		realPlayer.addSensorModel(vlineManager.getSensorModelState());
		realPlayer.setDynamicModel(vlineManager.getDynamicModel());
		realPlayer.addEstimator(est);
	}


	private static void prepareEstimatorSlave(Estimator est, VLinesModelsManager vlineManager) throws ConfiguratorException
	{
		Configurator conf = Configurator.getInstance();
		PropertiesHolder props = conf.getPropertiesHolder();

		vlineManager.setWorldMap(worldMap);
		vlineManager.getSensorModelMap().configure(props, "/vlinesnsmodel");
		vlineManager.getSensorModelState().configure(props, "/vlinesnsmodel");
		((VLinesDynModel) vlineManager.getDynamicModel()).configure(props, "/vlinedynmodel");
		vlineManager.configure(props, "/vlinemanager");

		vlineManager.getDynamicModel().setSensor(odoSensor);
		vlineManager.getSensorModelMap().setSensor(vlineManager.getSensorMap());
		vlineManager.getSensorModelState().setSensor(vlineManager.getSensorState());

		est.setDynamicModel(vlineManager.getDynamicModel(), odoSensor);
		est.addSensorModel(vlineManager.getSensorModelMap(), vlineManager.getSensorMap());
		est.addSensorModel(vlineManager.getSensorModelState(), vlineManager.getSensorState());

//		realPlayer.addSensor(vlineManager.getSensorMap());
//		realPlayer.addSensor(vlineManager.getSensorState());
		realPlayer.addSensorModel(vlineManager.getSensorModelMap());
		realPlayer.addSensorModel(vlineManager.getSensorModelState());
		realPlayer.setDynamicModel(vlineManager.getDynamicModel());
		realPlayer.addEstimator(est);
	}


	private static int [] arrayMap = null;
	private static int imgX0 = -1;
	private static int imgY0 = -1;
	private static int imgWidth = -1;
	private static int imgHeight = -1;
	public static void execute()
	{
		int itCount = 0;
		log.debug("execute:begin");
		realPlayer.resetPlayer();
		for (int idxStep = 0; idxStep < realPlayer.getNumberOfSteps(); idxStep++)
		{
			log.debug("execute:idxStep = " + idxStep);
			realPlayer.step();

			{
				Estimator est = (Estimator) listEstimators.get(0);
				AbstractDoubleVector mean = est.getMean();
				if (arrayMap == null)
				{
					Rectangle2D rect = worldMap.getBoundingBox();
					imgX0 = (int) (rect.getX());
					imgY0 = (int) (rect.getX());
					imgWidth = (int) (rect.getWidth() - imgX0);
					imgHeight = (int) (rect.getHeight() - imgY0);
					arrayMap = new int[imgHeight*imgWidth/100];
					log.debug("imgWidth: " + imgWidth);
					log.debug("imgHeight: " + imgHeight);
				}
				java.util.Arrays.fill(arrayMap, 0xFFFFFF);


				int x = (int) mean.getComponent(0), y = (int) mean.getComponent(1);
				ThreeBandPackedUtil.fillRect(x/10, y/10, 5, 5, 0xFF0000, arrayMap, imgWidth/10, imgHeight/10);

				for (int i = 3; i < mean.dimension(); i += 2)
				{
					x = (int) mean.getComponent(i); y = (int) mean.getComponent(i + 1);
					ThreeBandPackedUtil.fillRect(x/10, y/10, 5, 5, 0x0000FF, arrayMap, imgWidth/10, imgHeight/10);
				}
				Iterator itPointsVLine = worldMap.getVLines().iterator();
				while (itPointsVLine.hasNext())
				{
					VLineRef vline = (VLineRef) itPointsVLine.next();
					x = (int) vline.getX(); y = (int) vline.getY();
					ThreeBandPackedUtil.fillRect(x/10, y/10, 5, 5, 0x00FF00, arrayMap, imgWidth/10, imgHeight/10);
				}

				ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(arrayMap, imgWidth/10, imgHeight/10, BufferedImage.TYPE_INT_RGB), "map_" + itCount++ + ".bmp");
			}

		}
		log.debug("execute:end");
	}
}

