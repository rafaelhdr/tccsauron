package br.com.r4j.robosim.gui;

import java.awt.BorderLayout;
import java.util.HashMap;
import java.util.Iterator;

import javax.swing.Icon;
import javax.swing.JComponent;
import javax.swing.JMenuBar;
import javax.swing.JTabbedPane;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.robosim.ActionFileRobotPlayer;
import br.com.r4j.robosim.StatsComponent;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 */
public class StatsUI implements BlockUI
{
	private static Log log = LogFactory.getLog(StatsUI.class.getName());

	private JComponent contentPane = null;
	private JComponent glassPane = null;

	private JTabbedPane tabbPane = null;

	private ActionFileRobotPlayer robotPlayer = null;

	protected GUIStyleManager styleManager = null;

	private boolean bCanEstimate = false;
	
	
	private HashMap mapTab2BlockUI = null;


	public StatsUI()
	{
		log.debug("StatsUI");

		tabbPane = new JTabbedPane();
		
		mapTab2BlockUI = new HashMap();

		this.initActions();
	}


	public void addPane(BlockUI paneUI, String strTabName, Icon ico, String strDescription)
	{
		paneUI.setGlassPane(glassPane);
		tabbPane.addTab(strTabName, ico, paneUI.getContentPane(), strDescription);
		
		mapTab2BlockUI.put(strTabName, paneUI);
	}


	public GUIStyleManager getStyleManager()	{return styleManager;}
	public void setStyleManager(GUIStyleManager aStyleMan)
	{
		styleManager = aStyleMan;
	}


	private void initActions()
	{
		log.debug("initActions");
	}


	public void setGlassPane(JComponent glassPane)	{this.glassPane = glassPane;}

	public void setContentPane(JComponent contentPane)	{this.contentPane = contentPane;}
	public JComponent getContentPane()	{return this.contentPane;}
	public void setMenuBar(JMenuBar menuBar)	{}	


	public void build()
	{
		log.debug("build");
		
		Iterator itKeys = mapTab2BlockUI.keySet().iterator();
		while (itKeys.hasNext())
		{
			BlockUI blk = (BlockUI) mapTab2BlockUI.get(itKeys.next());
			blk.build();
		}
		
		tabbPane.setTabLayoutPolicy(JTabbedPane.WRAP_TAB_LAYOUT);
		this.getContentPane().setLayout(new BorderLayout());
		this.getContentPane().add(tabbPane, BorderLayout.CENTER);
	}


	public void adjustComponents()
	{
		Iterator itKeys = mapTab2BlockUI.keySet().iterator();
		while (itKeys.hasNext())
		{
			BlockUI blk = (BlockUI) mapTab2BlockUI.get(itKeys.next());
			blk.adjustComponents();
		}
	}


	public void stop()
	{
	}


	public void destroy()
	{
		Iterator itKeys = mapTab2BlockUI.keySet().iterator();
		while (itKeys.hasNext())
		{
			BlockUI blk = (BlockUI) mapTab2BlockUI.get(itKeys.next());
			blk.destroy();
		}
	}


	public void rebuild()
	{
		Iterator itKeys = mapTab2BlockUI.keySet().iterator();
		while (itKeys.hasNext())
		{
			BlockUI blk = (BlockUI) mapTab2BlockUI.get(itKeys.next());
			blk.rebuild();
		}
	}


	public void canProduceEstimates(boolean bCanEstimate)
	{
		this.bCanEstimate = bCanEstimate;
	}


	/**
	 * @param comp
	 */
	public void addEstimationStatisticsListener(StatsComponent comp)
	{
	}
}

