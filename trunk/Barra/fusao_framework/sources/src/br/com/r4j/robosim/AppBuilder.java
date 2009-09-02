package br.com.r4j.robosim;

import java.awt.Color;
import java.awt.Component;
import java.awt.Frame;
import java.awt.Point;
import java.awt.Shape;
import java.awt.event.ActionEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JComponent;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
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
import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.CSSLikeGUIStyleManager;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.gui.SplashScreen;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.impl.BaseEstimator;
import br.com.r4j.robosim.estimator.provider.EstimationObjectsProvider;
import br.com.r4j.robosim.gui.ActionPlayerControlUI;
import br.com.r4j.robosim.gui.ApplicationUI;
import br.com.r4j.robosim.gui.Legend;
import br.com.r4j.robosim.gui.OnlineRMSError;
import br.com.r4j.robosim.gui.OnlineUI;
import br.com.r4j.robosim.gui.RMSStatsUI;
import br.com.r4j.robosim.gui.RandomNoiseRMSStatsUI;
import br.com.r4j.robosim.gui.SimulatorUI;
import br.com.r4j.robosim.gui.StatsUI;
import br.com.r4j.robosim.gui.WorldUI;
import br.com.r4j.image.operation.threebandpacked.spatialfilter.SpatialFilterOneInputImageOp;


public class AppBuilder implements ConfiguratorListener
{
	private static Log log = LogFactory.getLog(AppBuilder.class.getName());
	private static Log logTime = LogFactory.getLog("time");

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

	private RobotPlayer rPlayer = null;

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

			int [] inDataTmp = new int[320*240];
			int [] tmpData = new int[inDataTmp.length];
			SpatialFilterOneInputImageOp closingOp = new SpatialFilterOneInputImageOp();
			closingOp.setFilter(new float [] {0, 1, 0,
											1, 1, 1,
											0, 1, 0},
									3, 3);
			long tmTm = System.currentTimeMillis();
			for (int i = 0; i < 100; i++)
					closingOp.operate(inDataTmp, tmpData, 320, 240);
			logTime.debug("time base: " + (System.currentTimeMillis() - tmTm)/100);

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
			Color.red,
			Color.blue,
			Color.green,
			Color.pink,
			Color.gray,
			Color.pink.darker()
		};
		arrayColorBorder = new Color[]
		{
			Color.yellow.darker(),
			Color.black,
			Color.yellow,
			Color.gray.darker(),
			Color.green.darker().darker().darker(),
			Color.black
		};
		arrayShape = new Shape []
		{
//			ShapesUtil.createRectangle(18, 6, true),
			ShapesUtil.rotateShape(ShapesUtil.createFourPointsStart(22, 4), Math.PI/4),
			ShapesUtil.createOval(5, 22, true),
			ShapesUtil.createOval(12, 12, true),
			ShapesUtil.createSpeedoShip(14, 20, 13, true),
			ShapesUtil.createOval(20, 12, true),
			ShapesUtil.createRectangle(20, 12, true),
			ShapesUtil.createTriangle(15, 15, 15, true),
			ShapesUtil.createOvalShip(20, 12, true)
		};


		RMSStatsUI statsRMS = new RMSStatsUI(statsUI);
		RandomNoiseRMSStatsUI statsRandomNoiseRMS = new RandomNoiseRMSStatsUI(statsUI);
		listStatsComponents.add(statsRMS);
		listStatsComponents.add(statsRandomNoiseRMS);

		worldMap = new WorldMap();
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
		GUIStyleManager styleMan = CSSLikeGUIStyleManager.getDefaultInstance();

		statsUI = new StatsUI();
		simulatorUI = new SimulatorUI();
		actionPlayerControlUI = new ActionPlayerControlUI();
		worldUI = new WorldUI();
		applicationUI = new  ApplicationUI();
		
		applicationUI.setStyleManager(styleMan);
		simulatorUI.setStyleManager(styleMan);
		statsUI.setStyleManager(styleMan);
		actionPlayerControlUI.setStyleManager(styleMan);
		worldUI.setStyleManager(styleMan);

		applicationUI.setWorldMap(worldMap);
		applicationUI.setLoadSimulationWizard(wizSim);

		// tem que setr mais um monte de panels para esse cara ...
		statsUI.setContentPane(applicationUI.getStatsContentPane());
		statsUI.setGlassPane(applicationUI.getStatsGlassPane());

		simulatorUI.setWorldUI(worldUI);
		simulatorUI.setplayerUI(actionPlayerControlUI);
		simulatorUI.setContentPane(applicationUI.getSimulatorContentPane());
		simulatorUI.setGlassPane(applicationUI.getSimulatorGlassPane());

		worldUI.setGlassPane(applicationUI.getSimulatorGlassPane());

		Iterator itStatsComponents = listStatsComponents.iterator();
		while (itStatsComponents.hasNext())
		{
			StatsComponent comp = (StatsComponent) itStatsComponents.next();
			BlockUI blk = comp.getUI(); 
			blk.setStyleManager(styleMan);
			statsUI.addPane(blk, comp.getShortName(), comp.getIcon(), comp.getDescription());
			statsUI.addEstimationStatisticsListener(comp);
		}

		worldMap.addChangeListener(worldUI);
		
		JMenuBar appMenuBar = applicationUI.getAppMenuBar();
		JMenu fileMenu = new JMenu("File");
		appMenuBar.add(fileMenu);
		ReloadEstimatorsAction actReload = new ReloadEstimatorsAction();
		fileMenu.add(actReload);

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
					"Problema configurando o Wizard",
					"Problema configurando o Wizard",
					JOptionPane.ERROR_MESSAGE);
		}
	}


	public void configurationDone(ConfiguratorEvent e)
	{
		{
			RobotPlayer robotPlayer = wizSim.getRobotPlayer();
			rPlayer = robotPlayer;
		}
		rPlayer.setInfoSink(worldUI);
		
		List listEst = rPlayer.getEstimators();
		List listSensor = rPlayer.getSensors();
		List listSensorModels = rPlayer.getSensorModels();

		OnlineRMSError rmsErr = new OnlineRMSError();
		
//		rPlayer.setInitialState(worldMap.getInitialLocalization(), worldMap.getInitialLocalizationCovar());
		rPlayer.setInitialState(worldMap.getInitialLocalizationEstimate(), worldMap.getInitialLocalizationCovar());
		rPlayer.setPlayRate(100);
/*
		List listEst = wizSim.getSelectedEstimators();
		List listSensor = wizSim.getUsedSensors();
		rPlayer.reset();

		Iterator itSensor = listSensor.iterator();
		while (itSensor.hasNext())
		{
			Sensor sens = (Sensor) itSensor.next();
			rPlayer.addSensor(sens);
		}

		Iterator itEst = listEst.iterator();
		while (itEst.hasNext())
		{
			Estimator est = (Estimator) itEst.next();
			rPlayer.addEstimator(est);
		}
//*/


		// Seta os Renderers para a interface WorldUI
		TreeMap mapRendererInfos = new TreeMap();
		{
			Iterator itLastRenderers = listLastRenderers.iterator();
			while (itLastRenderers.hasNext())
			{
				RendererListener rndr = (RendererListener) itLastRenderers.next();
				worldUI.removeRendererListener(rndr);
			}
			listLastRenderers.clear();

			listLastRenderers.add(worldMap);
			if (rPlayer instanceof RealTrackGenerator)
			{
				EstimatorRendererInfo realInfo = new EstimatorRendererInfo("Real");
				realInfo.setColorFill(Color.black);
				realInfo.setColorBorder(Color.red);
//				realInfo.setShape(ShapesUtil.rotateShape(ShapesUtil.createTriangle(20, 12, 20, true), -Math.PI/2));
				realInfo.setShape(ShapesUtil.rotateShape(ShapesUtil.createTriangle(Math.PI/3, (float) 13.5, Math.PI/3, true), -Math.PI/2));
				mapRendererInfos.put(realInfo, "Real");
				
				rmsErr.setRealTrackGenerator((RealTrackGenerator) rPlayer, realInfo, rPlayer);

				RobotTrack realTrackRenderer = new RobotTrack();
				realTrackRenderer.setEstimatorRendererInfo(realInfo);
				realTrackRenderer.setName("Tragetória Real");
				((RealTrackGenerator) rPlayer).addRobotTracker(realTrackRenderer);
				listLastRenderers.add((RendererListener) realTrackRenderer);
				rPlayer.addEstimatorRenderer(realTrackRenderer);

				RobotPose realTrackPose = new RobotPose();
				realTrackPose.setEstimatorRendererInfo(realInfo);
				realTrackPose.setName("Postura Real");
				((RealTrackGenerator) rPlayer).addRobotTracker(realTrackPose);
				listLastRenderers.add((RendererListener) realTrackPose);
				rPlayer.addEstimatorRenderer(realTrackPose);
			}
			
			Iterator itEst = listEst.iterator();
			int countInfos = 0;
			while (itEst.hasNext())
			{
				BaseEstimator est = (BaseEstimator) itEst.next();

				log.debug("est.getInstanceName(): " + est.getInstanceName());
				EstimatorRendererInfo estInfo = new EstimatorRendererInfo(est.getInstanceName());
				estInfo.setColorFill(arrayColorFill[countInfos]);
				estInfo.setColorBorder(arrayColorBorder[countInfos]);
				estInfo.setShape(arrayShape[countInfos]);
				mapRendererInfos.put(estInfo, est.getInstanceName());
				countInfos = (countInfos + 1)%arrayShape.length;
				
				rPlayer.setEstimatorInfo(est, estInfo);

				rmsErr.setEstimator(est, estInfo, rPlayer);
//*				
				RobotTrack estTrackRenderer = new RobotTrack();
				estTrackRenderer.setEstimatorRendererInfo(estInfo);
				estTrackRenderer.setName("Tragetória por " + est.getInstanceName());
				est.addRobotTracker(estTrackRenderer);
				listLastRenderers.add((RendererListener) estTrackRenderer);
				rPlayer.addEstimatorRenderer(estTrackRenderer);
//*/
				RobotCovariance estCovarRenderer = new RobotCovariance(est);
				estCovarRenderer.setEstimatorRendererInfo(estInfo);
				estCovarRenderer.setName("Covariância por " + est.getInstanceName());
				est.addRobotTracker(estCovarRenderer);
				listLastRenderers.add((RendererListener) estCovarRenderer);
				rPlayer.addEstimatorRenderer(estCovarRenderer);
				
				RobotPose estPoseRenderer = new RobotPose();
				estPoseRenderer.setEstimatorRendererInfo(estInfo);
				estPoseRenderer.setName("Postura por " + est.getInstanceName());
				est.addRobotTracker(estPoseRenderer);
				listLastRenderers.add((RendererListener) estPoseRenderer);
				rPlayer.addEstimatorRenderer(estPoseRenderer);

				Iterator itEstRenderers = est.getRenderers().iterator();
				while (itEstRenderers.hasNext())
				{
					RendererListener rndrLstnr = (RendererListener) itEstRenderers.next();
					if (rndrLstnr instanceof EstimatorRenderer)
					{
						((EstimatorRenderer) rndrLstnr).setEstimatorRendererInfo(estInfo);
						rPlayer.addEstimatorRenderer((EstimatorRenderer) rndrLstnr);
					}
					listLastRenderers.add(rndrLstnr);
				}

				if (est instanceof RobotPlayerListener)
					rPlayer.addRobotPlayerListener((RobotPlayerListener) est);
			}

			Iterator itSensor = listSensor.iterator();
			while (itSensor.hasNext())
			{
				Sensor sens = (Sensor) itSensor.next();
				if (sens instanceof RendererListener)
					listLastRenderers.add((RendererListener) sens);

				if (sens instanceof EstimatorRenderer)
					rPlayer.addEstimatorRenderer((EstimatorRenderer) sens);

				if (sens instanceof RobotPlayerListener)
					rPlayer.addRobotPlayerListener((RobotPlayerListener) sens);
			}

			itSensor = listSensorModels.iterator();
			while (itSensor.hasNext())
			{
				SensorModel sens = (SensorModel) itSensor.next();
				if (sens instanceof RendererListener)
					listLastRenderers.add((RendererListener) sens);

				if (sens instanceof EstimatorRenderer)
					rPlayer.addEstimatorRenderer((EstimatorRenderer) sens);

				if (sens instanceof RobotPlayerListener)
					rPlayer.addRobotPlayerListener((RobotPlayerListener) sens);
			}

			itLastRenderers = listLastRenderers.iterator();
			while (itLastRenderers.hasNext())
			{
				RendererListener rndr = (RendererListener) itLastRenderers.next();
				worldUI.addRendererListener(rndr);
			}
		}

		rPlayer.addRobotPlayerListener(worldUI);
		actionPlayerControlUI.setPlayer(rPlayer);
		simulatorUI.showPlayer(true);

		ArrayList listOnlineStats = new ArrayList();
		Legend leg = new Legend();

		listOnlineStats.add(leg);
		listOnlineStats.add(rmsErr);
		
		leg.setLocation(100, 100); leg.setSize(100, 100);
		rmsErr.setLocation(200, 100); rmsErr.setSize(100, 100);

		this.installOnlineStats(applicationUI.getSimulatorContentPane(), applicationUI.getSimulatorGlassPane(), listOnlineStats, mapRendererInfos);

		simulatorUI.rebuild();

		if (rPlayer instanceof EstimatesEngine)
		{
			Iterator itStatsComponents = listStatsComponents.iterator();
			while (itStatsComponents.hasNext())
			{
				StatsComponent comp = (StatsComponent) itStatsComponents.next();
				comp.setEstimatesEngine(rPlayer);
/*
				if (comp instanceof RMSStatsUI)
					((RMSStatsUI) comp).showLegends();
				if (comp instanceof RandomNoiseRMSStatsUI)
					((RandomNoiseRMSStatsUI) comp).showLegends();
//*/
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


	public void installOnlineStats(JComponent contentPane, JComponent glassPane, List listOnlineStats, Map mapRendererInfos)
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


	class ReloadEstimatorsAction extends AbstractAction
	{
		public ReloadEstimatorsAction()
		{
			super("Recarregar");
			this.putValue(Action.SHORT_DESCRIPTION, "Recarregar os Estimadores");
		}


		public void actionPerformed(ActionEvent e)
		{
			try
			{
				if (rPlayer != null)
					rPlayer.resetPlayer();

				Configurator conf = Configurator.getInstance();
				PropertiesHolder props = conf.getPropertiesHolder();

				buildComponents();
				configure(props);
					
				wizSim.setParentFrame(applicationUI.getRootFrame());
				try
				{
					wizSim.configure(worldMap);
				}
				catch (ConfiguratorException ex)
				{
					log.error("erro", ex);
					JOptionPane.showMessageDialog(null, 
							"Problema configurando o Wizard",
							"Problema configurando o Wizard",
							JOptionPane.ERROR_MESSAGE);
				}
			}
			catch (ConfiguratorException ex)
			{
				log.error("erro", ex);
				JOptionPane.showMessageDialog(null, 
						"Problema configurando o Wizard",
						"Problema configurando o Wizard",
						JOptionPane.ERROR_MESSAGE);
			}
		}
	}
}
