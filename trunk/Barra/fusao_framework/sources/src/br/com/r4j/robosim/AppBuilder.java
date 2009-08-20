package br.com.r4j.robosim;

import java.awt.Color;
import java.awt.Component;
import java.awt.Frame;
import java.awt.Point;
import java.awt.Shape;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputAdapter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.configurator.WebStartConfigurator;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.gui.SplashScreen;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.provider.EstimationObjectsProvider;
import br.com.r4j.robosim.gui.ActionPlayerControlUI;
import br.com.r4j.robosim.gui.ApplicationUI;
import br.com.r4j.robosim.gui.Legend;
import br.com.r4j.robosim.gui.OnlineRMSError;
import br.com.r4j.robosim.gui.OnlineUI;
import br.com.r4j.robosim.gui.SimulatorUI;
import br.com.r4j.robosim.gui.StatsUI;
import br.com.r4j.robosim.gui.WorldUI;


public class AppBuilder implements ConfiguratorListener
{
	private static Log log = LogFactory.getLog(AppBuilder.class.getName());

	private WorldUI worldUI = null;
	private StatsUI statsUI = null;
	private SimulatorUI simulatorUI = null;
	private ApplicationUI applicationUI = null;
	private ActionPlayerControlUI actionPlayerControlUI = null;

//	private RobotRenderer robotRenderer = null;

	private List listLastRenderers = null;

	private EstimationObjectsProvider estProvider = null;
	private LoadSimulationWizard wizSim = null;
	private WorldMap worldMap = null;

	private List listStatsComponents = null;

	private Color [] arrayColorFill = null;
	private Color [] arrayColorBorder = null;
	private Shape [] arrayShape = null;


	public static void main(String [] args)
	{
//		new br.com.r4j.image.operation.imageop.GradientOp();
		try
		{
			WebStartConfigurator.createConfigurator("conf/conf.xml");

			Configurator conf = Configurator.getInstance();
			PropertiesHolder props = conf.getPropertiesHolder();

			SplashScreen splash = new SplashScreen(null, ImageUtil.loadImageAsResource("/" + AppBuilder.class.getName().replace('.', '/') + "/splashImg.jpg"), 5000);
			splash.toFront();

			AppBuilder app = new AppBuilder();
			app.buildComponents();
			app.configure(props);
			app.buildUI();
			splash.kill();
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


	public AppBuilder()
	{
		listLastRenderers = new ArrayList();
		listStatsComponents = new ArrayList();

		arrayColorFill = new Color[]
		{
			Color.yellow.darker(),
			Color.blue,
			Color.pink,
			Color.green,
			Color.gray
		};
		arrayColorBorder = new Color[]
		{
			Color.pink.darker(),
			Color.black,
			Color.yellow,
			Color.gray.darker(),
			Color.green.darker().darker().darker()
		};
		arrayShape = new Shape []
		{
			ShapesUtil.createSpeedoShip(15, 20, 12, true),
			ShapesUtil.createOval(20, 12, true),
			ShapesUtil.createTriangle(15, 15, 15, true),
			ShapesUtil.createRectangle(20, 12, true),
			ShapesUtil.createOvalShip(20, 12, true),
		};


//		listStatsComponents.add(...);
	}


	
	public Frame getTopWindow()
	{
		return applicationUI.getRootFrame();
	}

	
	public void buildComponents()
	{
		// Configurar com arquivo de cofn. Esse é o responsável pr sair criando tudo ...
		estProvider = new EstimationObjectsProvider();

		// Configurar passando parâmetros padrões.
		wizSim = new LoadSimulationWizard();
		wizSim.addConfiguratorListener(this);
		wizSim.setProvider(estProvider);

		worldMap = new WorldMap();
	}


	public void configure(PropertiesHolder props) throws ConfiguratorException
	{
		estProvider.loadConfiguration(props, "robosim/estimator_provider/");

		File flReading = props.getFileProperty("robosim/readings");
		if (flReading != null)
			wizSim.setFileRealReadings(flReading);
		File flRealPose = props.getFileProperty("robosim/realpose");
		if (flRealPose != null)
			wizSim.setFileRealPose(flRealPose);
		File flWorld = props.getFileProperty("robosim/worldmap");
		if (flWorld != null)
			wizSim.setFileWorldMap(flWorld);
		File flActions = props.getFileProperty("robosim/actions");
		if (flActions != null)
			wizSim.setFileActions(flActions);
	}


	public void buildUI()
	{
		statsUI = new StatsUI();
		simulatorUI = new SimulatorUI();
		actionPlayerControlUI = new ActionPlayerControlUI();
		worldUI = new WorldUI();
		applicationUI = new  ApplicationUI();

		applicationUI.setWorldMap(worldMap);
		applicationUI.setLoadSimulationWizard(wizSim);

		// tem que setr mais um monte de panels para esse cara ...
		statsUI.setContentPane(applicationUI.getStatsContentPane());

		simulatorUI.setWorldUI(worldUI);
		simulatorUI.setplayerUI(actionPlayerControlUI);
		simulatorUI.setContentPane(applicationUI.getSimulatorContentPane());
		simulatorUI.setGlassPane(applicationUI.getSimulatorGlassPane());

		worldUI.setGlassPane(applicationUI.getSimulatorGlassPane());

		Iterator itStatsComponents = listStatsComponents.iterator();
		while (itStatsComponents.hasNext())
		{
			StatsComponent comp = (StatsComponent) itStatsComponents.next();
			statsUI.addPane(comp.getUI(), comp.getShortName(), comp.getIcon(), comp.getDescription());
		}

		worldMap.addChangeListener(worldUI);

		worldUI.build();
		actionPlayerControlUI.build();
		simulatorUI.build();
		statsUI.build();

		applicationUI.build();
		applicationUI.showUI();

		wizSim.setParentFrame(applicationUI.getRootFrame());
		try
		{
			wizSim.configure(worldMap);
		}
		catch (ConfiguratorException e)
		{
			log.error("erro", e);
			JOptionPane.showMessageDialog(null, 
					"Problema configruando o Wizard",
					"Problema configruando o Wizard",
					JOptionPane.ERROR_MESSAGE);
		}
	}


	public void configurationDone(ConfiguratorEvent e)
	{
		RobotPlayer robotPlayer = wizSim.getRobotPlayer();
		List listEst = robotPlayer.getEstimators();
		List listSensor = robotPlayer.getSensors();
		List listSensorModels = robotPlayer.getSensorModels();

		OnlineRMSError rmsErr = new OnlineRMSError();
		
		robotPlayer.setInitialState(worldMap.getInitialLocalization(), worldMap.getInitialLocalizationCovar());
		robotPlayer.setPlayRate(100);
/*
		List listEst = wizSim.getSelectedEstimators();
		List listSensor = wizSim.getUsedSensors();
		robotPlayer.reset();

		Iterator itSensor = listSensor.iterator();
		while (itSensor.hasNext())
		{
			Sensor sens = (Sensor) itSensor.next();
			robotPlayer.addSensor(sens);
		}

		Iterator itEst = listEst.iterator();
		while (itEst.hasNext())
		{
			Estimator est = (Estimator) itEst.next();
			robotPlayer.addEstimator(est);
		}
//*/


		// Seta os Renderers para a interface WorldUI
		HashMap mapRendererInfos = new HashMap();
		{
			Iterator itLastRenderers = listLastRenderers.iterator();
			while (itLastRenderers.hasNext())
			{
				RendererListener rndr = (RendererListener) itLastRenderers.next();
				worldUI.removeRendererListener(rndr);
			}
			listLastRenderers.clear();

			listLastRenderers.add(worldMap);
			if (robotPlayer instanceof RealTrackGenerator)
			{
				EstimatorRendererInfo realInfo = new EstimatorRendererInfo();
				realInfo.setColorFill(Color.black);
				realInfo.setColorBorder(Color.red);
				realInfo.setShape(ShapesUtil.rotateShape(ShapesUtil.createTriangle(20, 12, 20, true), -Math.PI/2));
				mapRendererInfos.put(realInfo, "Real");
				
				rmsErr.setRealTrackGenerator((RealTrackGenerator) robotPlayer, realInfo, robotPlayer);

				RobotTrack realTrackRenderer = new RobotTrack();
				realTrackRenderer.setEstimatorRendererInfo(realInfo);
				realTrackRenderer.setName("Tragetória Real");
				((RealTrackGenerator) robotPlayer).addRobotTracker(realTrackRenderer);
				listLastRenderers.add((RendererListener) realTrackRenderer);
				robotPlayer.addEstimatorRenderer(realTrackRenderer);

				RobotPose realTrackPose = new RobotPose();
				realTrackPose.setEstimatorRendererInfo(realInfo);
				realTrackPose.setName("Postura Real");
				((RealTrackGenerator) robotPlayer).addRobotTracker(realTrackPose);
				listLastRenderers.add((RendererListener) realTrackPose);
				robotPlayer.addEstimatorRenderer(realTrackPose);
			}
			
			Iterator itEst = listEst.iterator();
			int countInfos = 0;
			while (itEst.hasNext())
			{
				Estimator est = (Estimator) itEst.next();

				EstimatorRendererInfo estInfo = new EstimatorRendererInfo();
				estInfo.setColorFill(arrayColorFill[countInfos]);
				estInfo.setColorBorder(arrayColorBorder[countInfos]);
				estInfo.setShape(arrayShape[countInfos]);
				mapRendererInfos.put(estInfo, est.getName());
				countInfos = (countInfos + 1)%arrayShape.length;

				rmsErr.setEstimator(est, estInfo, robotPlayer);
//*				
				RobotTrack estTrackRenderer = new RobotTrack();
				estTrackRenderer.setEstimatorRendererInfo(estInfo);
				estTrackRenderer.setName("Tragetória por " + est.getName());
				est.addRobotTracker(estTrackRenderer);
				listLastRenderers.add((RendererListener) estTrackRenderer);
				robotPlayer.addEstimatorRenderer(estTrackRenderer);
//*/
				RobotCovariance estCovarRenderer = new RobotCovariance(est);
				estCovarRenderer.setEstimatorRendererInfo(estInfo);
				estCovarRenderer.setName("Covariância por " + est.getName());
				est.addRobotTracker(estCovarRenderer);
				listLastRenderers.add((RendererListener) estCovarRenderer);
				robotPlayer.addEstimatorRenderer(estCovarRenderer);
				
				RobotPose estPoseRenderer = new RobotPose();
				estPoseRenderer.setEstimatorRendererInfo(estInfo);
				estPoseRenderer.setName("Postura por " + est.getName());
				est.addRobotTracker(estPoseRenderer);
				listLastRenderers.add((RendererListener) estPoseRenderer);
				robotPlayer.addEstimatorRenderer(estPoseRenderer);

				Iterator itEstRenderers = est.getRenderers().iterator();
				while (itEstRenderers.hasNext())
				{
					RendererListener rndrLstnr = (RendererListener) itEstRenderers.next();
					if (rndrLstnr instanceof EstimatorRenderer)
					{
						((EstimatorRenderer) rndrLstnr).setEstimatorRendererInfo(estInfo);
						robotPlayer.addEstimatorRenderer((EstimatorRenderer) rndrLstnr);
					}
					listLastRenderers.add(rndrLstnr);
				}

				if (est instanceof RobotPlayerListener)
					robotPlayer.addRobotPlayerListener((RobotPlayerListener) est);
			}

			Iterator itSensor = listSensor.iterator();
			while (itSensor.hasNext())
			{
				Sensor sens = (Sensor) itSensor.next();
				if (sens instanceof RendererListener)
					listLastRenderers.add((RendererListener) sens);

				if (sens instanceof EstimatorRenderer)
					robotPlayer.addEstimatorRenderer((EstimatorRenderer) sens);

				if (sens instanceof RobotPlayerListener)
					robotPlayer.addRobotPlayerListener((RobotPlayerListener) sens);
			}

			itSensor = listSensorModels.iterator();
			while (itSensor.hasNext())
			{
				SensorModel sens = (SensorModel) itSensor.next();
				if (sens instanceof RendererListener)
					listLastRenderers.add((RendererListener) sens);

				if (sens instanceof EstimatorRenderer)
					robotPlayer.addEstimatorRenderer((EstimatorRenderer) sens);

				if (sens instanceof RobotPlayerListener)
					robotPlayer.addRobotPlayerListener((RobotPlayerListener) sens);
			}

			itLastRenderers = listLastRenderers.iterator();
			while (itLastRenderers.hasNext())
			{
				RendererListener rndr = (RendererListener) itLastRenderers.next();
				worldUI.addRendererListener(rndr);
			}
		}

		robotPlayer.addRobotPlayerListener(worldUI);
		if (robotPlayer instanceof ActionFileRobotPlayer)
		{
			actionPlayerControlUI.setPlayer((ActionFileRobotPlayer) robotPlayer);
			simulatorUI.showPlayer(true);
		}
		else
		{
			simulatorUI.showPlayer(false);
		}


		ArrayList listOnlineStats = new ArrayList();
		Legend leg = new Legend();

		listOnlineStats.add(leg);
		listOnlineStats.add(rmsErr);
		
		leg.setLocation(100, 100); leg.setSize(100, 100);
		rmsErr.setLocation(200, 100); rmsErr.setSize(100, 100);

		this.installOnlineStats(applicationUI.getSimulatorContentPane(), applicationUI.getSimulatorGlassPane(), listOnlineStats, mapRendererInfos);

		simulatorUI.rebuild();


		if (robotPlayer instanceof EstimatesEngine)
		{
			Iterator itStatsComponents = listStatsComponents.iterator();
			while (itStatsComponents.hasNext())
			{
				StatsComponent comp = (StatsComponent) itStatsComponents.next();
				comp.setEstimatesEngine(robotPlayer);
			}

			statsUI.canProduceEstimates(true);
		}
		else
		{
			statsUI.canProduceEstimates(false);
		}
		statsUI.rebuild();

//		applicationUI.hideUI();
//		applicationUI.showUI();

		worldUI.setCanvasZoom(worldMap.getBoundingBox(), 10, 10);
	}

	public void installOnlineStats(JComponent contentPane, JComponent glassPane, List listOnlineStats, HashMap mapRendererInfos)
	{
		glassPane.setVisible(true);
		glassPane.setLayout(null);
		glassPane.removeAll();
		
		Iterator itOnlineUIs = listOnlineStats.iterator();
		while (itOnlineUIs.hasNext())
		{
			OnlineUI onUi = (OnlineUI) itOnlineUIs.next();
			onUi.setRendererInfos(mapRendererInfos);
			onUi.resetUI();
			glassPane.add(onUi.getComponent());
			onUi.getComponent().addMouseMotionListener(new MouseInputAdapter() {
				public void mouseDragged(MouseEvent e) {
					Component compSource = (Component) e.getSource();
					e.translatePoint(compSource.getLocation().x, compSource.getLocation().y);
					compSource.setLocation(e.getPoint());
				}
			});
		}
		
		MouseListener [] arrayListMouse = glassPane.getMouseListeners();
		for (int i = 0; i < arrayListMouse.length; i++)
			glassPane.removeMouseListener(arrayListMouse[i]);
		MouseMotionListener [] arrayListMotion = glassPane.getMouseMotionListeners();
		for (int i = 0; i < arrayListMotion.length; i++)
			glassPane.removeMouseMotionListener(arrayListMotion[i]);

		RedispatchMouseEvents mouseRedisp = new RedispatchMouseEvents(contentPane, glassPane);
		glassPane.addMouseListener(mouseRedisp);
		glassPane.addMouseMotionListener(mouseRedisp);
	}
	
	
	class RedispatchMouseEvents implements MouseListener, MouseMotionListener
	{
		private Component glassPane = null;
		private Component contentPane = null;
		 
		public RedispatchMouseEvents(Component contentPane, Component glassPane)
		{
			this.contentPane = contentPane;
			this.glassPane = glassPane;
		}

		// catch all mouse events and redispatch them
		public void mouseMoved(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseDragged(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseClicked(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseEntered(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseExited(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mousePressed(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseReleased(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}


		private void redispatchMouseEvent(MouseEvent e, boolean repaint)
		{
			// get the mouse click point relative to the content pane
			Point containerPoint = SwingUtilities.convertPoint(glassPane, e.getPoint(), contentPane);

			// find the component that under this point
			Component component = SwingUtilities.getDeepestComponentAt(
													contentPane,
													containerPoint.x,
													containerPoint.y);

				// return if nothing was found
				if (component == null)
					return;

				// convert point relative to the target component
				Point componentPoint = SwingUtilities.convertPoint(glassPane, e.getPoint(), component);

				// redispatch the event
				component.dispatchEvent(new MouseEvent(component,
												e.getID(),
												e.getWhen(),
												e.getModifiers(),
												componentPoint.x,
												componentPoint.y,
												e.getClickCount(),
												e.isPopupTrigger(),
												e.getButton()));
		}
	}
}
