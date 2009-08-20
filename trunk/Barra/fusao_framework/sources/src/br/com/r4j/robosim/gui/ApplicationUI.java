package br.com.r4j.robosim.gui;

import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.io.IOException;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JComponent;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.GUIUtil;
import br.com.r4j.gui.JAppDesktopPane;
import br.com.r4j.gui.JAppInternalFrame;
import br.com.r4j.robosim.LoadSimulationWizard;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.gui.actions.FileAction;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 *
 */
public class ApplicationUI implements BlockUI, ComponentListener
{
	private static Log log = LogFactory.getLog(ApplicationUI.class.getName());

	private JComponent contentPane = null;

	private JAppDesktopPane desktopPane = null;
	private JAppInternalFrame simulatorFrame = null;
	private JAppInternalFrame statsFrame = null;
	private JMenuBar simulatorMenuBar = null;
	private JMenuBar statsMenuBar = null;
	private JMenuBar appMenuBar = null;

	private Action actReloadSimulation = null;
	private Action actOpenMapFile = null;

	protected GUIStyleManager styleManager = null;

	private LoadSimulationWizard wizSim = null;
	private WorldMap wordlMap = null;


	public ApplicationUI()
	{
		log.debug("ApplicationUI");

		desktopPane = new JAppDesktopPane();
		simulatorFrame = new JAppInternalFrame();
		statsFrame = new JAppInternalFrame();

		this.initActions();

		simulatorMenuBar = new JMenuBar();
		statsMenuBar = new JMenuBar();
		appMenuBar = new JMenuBar();
	}


	public GUIStyleManager getStyleManager()	{return styleManager;}
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	private void initActions()
	{
		log.debug("initActions");
		actReloadSimulation = new ReloadSimulationAction();
		actOpenMapFile = new FileAction("Trocar Arquivo do Mapa", this, "setMapFile");
	}


	public void setContentPane(JComponent contentPane)
	{
		//this.contentPane = contentPane;
	}
	
	
	public JComponent getContentPane()
	{
		if (contentPane == null)
			contentPane = (JComponent) desktopPane.getContentPane();
		return this.contentPane;
	}
	public void setMenuBar(JMenuBar menuBar)	{}
	public Frame getRootFrame()	{return desktopPane;}


	public void build()
	{
		log.debug("build");

		desktopPane.showStatusArea(false);

		desktopPane.getContentPane().add(simulatorFrame);
		simulatorFrame.setStyleManager(this.getStyleManager());
		simulatorFrame.setClosable(false);
		simulatorFrame.pack();
        simulatorFrame.setSize(800, 600);
		simulatorFrame.setTitle("Visão do Robô");

		desktopPane.getContentPane().add(statsFrame);
		statsFrame.setStyleManager(this.getStyleManager());
		statsFrame.setClosable(false);
		statsFrame.pack();
        statsFrame.setSize(600, 400);
		statsFrame.setTitle("Estatísticas");

		desktopPane.showStatus("iniciado");
		desktopPane.getDesktopPane().getDesktopManager().activateFrame(simulatorFrame);

		statsFrame.setJMenuBar(statsMenuBar);
		desktopPane.setJMenuBar(simulatorMenuBar);
		simulatorFrame.setJMenuBar(appMenuBar);
		
		simulatorFrame.addComponentListener(this);

		log.debug("build:end");
	}


	public void adjustComponents()
	{
		log.debug("adjustComponents");
	}


	public void hideUI()
	{
		desktopPane.setVisible(false);
	}


	public void showUI()
	{
		log.debug("showUI");
//		desktopPane.show();
//		this.getContentPane().setVisible(true);
		desktopPane.setVisible(true);
		desktopPane.getContentPane().invalidate();
		desktopPane.getContentPane().validate();
		
		simulatorFrame.pack();
		statsFrame.pack();
		
		statsFrame.setSize(500, 500);
	}

	
	public void stop()
	{
	}


	public void destroy()
	{
	}


	public JMenuBar getStatsMenuBar()	{return statsMenuBar;}
	public JMenuBar getSimulatorMenuBar()	{return simulatorMenuBar;}

	public JComponent getStatsContentPane()	{return (JComponent) statsFrame.getContentPane();}
	public JComponent getStatsGlassPane()	{return (JComponent) statsFrame.getGlassPane();}
	public JComponent getSimulatorContentPane()	{return (JComponent) simulatorFrame.getContentPane();}
	public JComponent getSimulatorGlassPane()	{return (JComponent) simulatorFrame.getGlassPane();}

	public void setLoadSimulationWizard(LoadSimulationWizard wizSim)	{this.wizSim = wizSim;}
	public void setWorldMap(WorldMap wordlMap)	{this.wordlMap = wordlMap;}
	public void setMapFile(String strFile) throws IOException	{wordlMap.setMapFile(strFile);}


	class ReloadSimulationAction extends AbstractAction
	{
		public ReloadSimulationAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "play16.gif", "Play", ReloadSimulationAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Play");
		}


		public void actionPerformed(ActionEvent e)
		{
			try
			{
				wizSim.configure(wordlMap);
			}
			catch (ConfiguratorException ex)
			{
				log.error("erro", ex);
				JOptionPane.showMessageDialog(null, 
						"Problema configruando o Wizard",
						"Problema configruando o Wizard",
						JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.gui.BlockUI#rebuild()
	 */
	public void rebuild()
	{
		JComponent comp = this.getContentPane();
		comp.removeAll();
		this.build(); 
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentHidden(java.awt.event.ComponentEvent)
	 */
	public void componentHidden(ComponentEvent e)
	{
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentMoved(java.awt.event.ComponentEvent)
	 */
	public void componentMoved(ComponentEvent e)
	{
		getSimulatorGlassPane().setVisible(true);
		
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentResized(java.awt.event.ComponentEvent)
	 */
	public void componentResized(ComponentEvent e)
	{
		getSimulatorGlassPane().setVisible(true);
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentShown(java.awt.event.ComponentEvent)
	 */
	public void componentShown(ComponentEvent e)
	{
		getSimulatorGlassPane().setVisible(true);
	}
}
