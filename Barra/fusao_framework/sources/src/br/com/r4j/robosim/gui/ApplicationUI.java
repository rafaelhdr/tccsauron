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
 * @modelguid {0C947AD6-F6E5-443B-B2B6-71B39DF10A96}
 */
public class ApplicationUI implements BlockUI, ComponentListener
{
	/** @modelguid {9B9EB7E2-0697-445A-B4C9-B5B8703577BB} */
	private static Log log = LogFactory.getLog(ApplicationUI.class.getName());

	/** @modelguid {BF37223B-28BA-4579-8DBF-7BA5568EDB94} */
	private JComponent contentPane = null;

	/** @modelguid {4334C730-F107-4CF5-8E61-B4DA34BDAC91} */
	private JAppDesktopPane desktopPane = null;
	/** @modelguid {7B8B5CBC-5F48-452D-B21D-57616C300872} */
	private JAppInternalFrame simulatorFrame = null;
	/** @modelguid {69074752-E3F7-4F46-98B9-F4A34AA4600F} */
	private JAppInternalFrame statsFrame = null;
	/** @modelguid {CD242C2C-8FAB-4FFE-A7A8-11CA18BCB353} */
	private JMenuBar simulatorMenuBar = null;
	/** @modelguid {CCAB509E-452F-4B49-B133-11EA696ADA43} */
	private JMenuBar statsMenuBar = null;
	/** @modelguid {4E90F467-AF43-4F8F-9FAF-2D25AE6A711D} */
	private JMenuBar appMenuBar = null;

	/** @modelguid {42CFBC79-8BD1-4D1C-886A-910D266D9BD1} */
	private Action actReloadSimulation = null;
	/** @modelguid {94BFCE1A-C09A-441E-A126-D403F2D4C15E} */
	private Action actOpenMapFile = null;

	/** @modelguid {25DD18AF-3486-422C-BE2D-43881ACF2754} */
	protected GUIStyleManager styleManager = null;

	/** @modelguid {503C6003-5176-4FE9-875D-38EDA65B54BA} */
	private LoadSimulationWizard wizSim = null;
	/** @modelguid {6C3230A6-E4AE-4345-9445-33C72AF3A06A} */
	private WorldMap wordlMap = null;


	/** @modelguid {957D8984-1E3B-4E2A-9106-E39D65447DC6} */
	public ApplicationUI()
	{
		log.debug("ApplicationUI");

		desktopPane = new JAppDesktopPane();
		statsFrame = new JAppInternalFrame();
		simulatorFrame = new JAppInternalFrame();

		this.initActions();

		simulatorMenuBar = new JMenuBar();
		statsMenuBar = new JMenuBar();
		appMenuBar = new JMenuBar();
	}


	/** @modelguid {7B02E967-0351-480E-A73F-8C0CA87808AF} */
	public GUIStyleManager getStyleManager()	{return styleManager;}
	/** @modelguid {5BE7F58F-9245-4DD0-A91C-2F833B2D4C06} */
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	/** @modelguid {8C2ABF6E-9C2D-4808-848B-C56027B6D4C6} */
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

		this.getStyleManager().prepareRootFrame(getRootFrame());
		
		desktopPane.getContentPane().add(statsFrame);
		statsFrame.setStyleManager(this.getStyleManager());
		statsFrame.setClosable(false);
		statsFrame.pack();
		statsFrame.setSize(600, 400);
		statsFrame.setTitle("Estatísticas");

		desktopPane.getContentPane().add(simulatorFrame);
		simulatorFrame.setStyleManager(this.getStyleManager());
		simulatorFrame.setClosable(false);
		simulatorFrame.pack();
        simulatorFrame.setSize(800, 600);
		simulatorFrame.setTitle("Visão do Robô");

		desktopPane.showStatus("iniciado");
		desktopPane.getDesktopPane().getDesktopManager().activateFrame(simulatorFrame);
//		desktopPane.getDesktopPane().getDesktopManager().activateFrame(statsFrame);

		statsFrame.setJMenuBar(statsMenuBar);
		desktopPane.setJMenuBar(appMenuBar);
		simulatorFrame.setJMenuBar(simulatorMenuBar);
		
		statsFrame.addComponentListener(this);
		simulatorFrame.addComponentListener(this);

		log.debug("build:end");
	}


	public void adjustComponents()
	{
		log.debug("adjustComponents");
	}


	/** @modelguid {C7BCB7C4-203F-45D4-88D6-BF021097D06D} */
	public void hideUI()
	{
		desktopPane.setVisible(false);
	}


	/** @modelguid {03D18C5B-985B-48D9-9085-88B71CFB34FB} */
	public void showUI()
	{
		log.debug("showUI");
//		desktopPane.show();
//		this.getContentPane().setVisible(true);
		desktopPane.setVisible(true);
		desktopPane.getContentPane().invalidate();
		desktopPane.getContentPane().validate();
		
		statsFrame.pack();
		simulatorFrame.pack();
		
		statsFrame.setSize(500, 500);
		simulatorFrame.setSize(500, 500);
	}

	
	/** @modelguid {2FDB5285-825C-4AF3-B8AF-185C9CA5A8BC} */
	public void stop()
	{
	}


	/** @modelguid {04500A72-CBF2-4FC5-A232-454CAE573463} */
	public void destroy()
	{
	}


	/** @modelguid {18F4EBA7-7DF4-4BE2-826F-70910D94FCB8} */
	public JMenuBar getStatsMenuBar()	{return statsMenuBar;}
	
	public JMenuBar getAppMenuBar()	{return appMenuBar;}
	
	/** @modelguid {4CFF2FFB-18C3-44E2-89C5-3A57809475CE} */
	public JMenuBar getSimulatorMenuBar()	{return simulatorMenuBar;}

	/** @modelguid {887D6EF6-ACA5-4A09-9E8A-8E2F22C4C66F} */
	public JComponent getStatsContentPane()	{return (JComponent) statsFrame.getContentPane();}
	/** @modelguid {0027FD32-80EA-4EF0-AE87-C9244FBB0E75} */
	public JComponent getStatsGlassPane()	{return (JComponent) statsFrame.getGlassPane();}
	/** @modelguid {9A529952-8F40-4C4F-8CF1-C378C1C79D38} */
	public JComponent getSimulatorContentPane()	{return (JComponent) simulatorFrame.getContentPane();}
	/** @modelguid {B49558BC-5CA7-4170-89B1-B6830D60CF29} */
	public JComponent getSimulatorGlassPane()	{return (JComponent) simulatorFrame.getGlassPane();}

	/** @modelguid {A21980FF-A2C3-49E8-957F-8784215238A8} */
	public void setLoadSimulationWizard(LoadSimulationWizard wizSim)	{this.wizSim = wizSim;}
	/** @modelguid {F8C3D7F1-63E4-469C-A28A-36529727261B} */
	public void setWorldMap(WorldMap wordlMap)	{this.wordlMap = wordlMap;}
	/** @modelguid {C585F788-7FD9-44D4-85CF-59DBCFD3DB3F} */
	public void setMapFile(String strFile) throws IOException	{wordlMap.setMapFile(strFile);}


	/** @modelguid {0EBA8F2C-01E9-4116-9E3E-E229B9D9FA38} */
	class ReloadSimulationAction extends AbstractAction
	{
		/** @modelguid {0819608F-3DB9-47C9-8EB8-37278D9035E5} */
		public ReloadSimulationAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "play16.gif", "Play", ReloadSimulationAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Play");
		}


		/** @modelguid {759FE821-FF62-417C-8B9D-C47ACC0D19C5} */
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
	 * @modelguid {04652447-2FF2-4D86-BBCE-C63C1BACA1AA}
	 */
	public void rebuild()
	{
		JComponent comp = this.getContentPane();
		comp.removeAll();
		this.build(); 
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentHidden(java.awt.event.ComponentEvent)
	 * @modelguid {45B76F91-274D-4FD1-AB68-6074BEA342D6}
	 */
	public void componentHidden(ComponentEvent e)
	{
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentMoved(java.awt.event.ComponentEvent)
	 * @modelguid {6E55EBF8-CEB9-4AC4-AD6F-E99E70242039}
	 */
	public void componentMoved(ComponentEvent e)
	{
		getSimulatorGlassPane().setVisible(true);
		getStatsGlassPane().setVisible(true);
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentResized(java.awt.event.ComponentEvent)
	 * @modelguid {C0D1EDC2-EF58-4530-AADC-8BF97C422D68}
	 */
	public void componentResized(ComponentEvent e)
	{
		getSimulatorGlassPane().setVisible(true);
		getStatsGlassPane().setVisible(true);
	}


	/* (non-Javadoc)
	 * @see java.awt.event.ComponentListener#componentShown(java.awt.event.ComponentEvent)
	 * @modelguid {92BD6AD3-0755-420C-B331-947FB7F4A7FD}
	 */
	public void componentShown(ComponentEvent e)
	{
		getSimulatorGlassPane().setVisible(true);
		getStatsGlassPane().setVisible(true);
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.gui.BlockUI#setGlassPane(javax.swing.JComponent)
	 */
	public void setGlassPane(JComponent glassPane)
	{
	}
}
