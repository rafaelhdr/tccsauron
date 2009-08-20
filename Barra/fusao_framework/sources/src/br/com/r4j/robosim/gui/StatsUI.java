package br.com.r4j.robosim.gui;

import javax.swing.Icon;
import javax.swing.JComponent;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.robosim.ActionFileRobotPlayer;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 *
 */
public class StatsUI implements BlockUI
{
	private static Log log = LogFactory.getLog(StatsUI.class.getName());

	private JComponent contentPane = null;

	private JTabbedPane tabbPane = null;

	private ActionFileRobotPlayer robotPlayer = null;

	protected GUIStyleManager styleManager = null;

	private boolean bCanEstimate = false;


	public StatsUI()
	{
		log.debug("StatsUI");

		tabbPane = new JTabbedPane();

		this.initActions();
	}


	public void addPane(BlockUI paneUI, String strTabName, Icon ico, String strDescription)
	{
		tabbPane.addTab(strTabName, ico, paneUI.getContentPane(), strDescription);
	}


	public GUIStyleManager getStyleManager()	{return styleManager;}
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	private void initActions()
	{
		log.debug("initActions");
	}


	public void setContentPane(JComponent contentPane)	{this.contentPane = contentPane;}
	public JComponent getContentPane()
	{
		if (contentPane == null)
			contentPane = new JPanel();  
		return this.contentPane;
	}
	public void setMenuBar(JMenuBar menuBar)	{}	


	public void build()
	{
		log.debug("build");
		tabbPane.setTabLayoutPolicy(JTabbedPane.WRAP_TAB_LAYOUT);
		this.getContentPane().add(tabbPane);
	}


	public void adjustComponents()
	{
	}



	public void stop()
	{
	}


	public void destroy()
	{
	}


	public void rebuild()
	{
	}


	public void canProduceEstimates(boolean bCanEstimate)
	{
		this.bCanEstimate = bCanEstimate;
	}
}

