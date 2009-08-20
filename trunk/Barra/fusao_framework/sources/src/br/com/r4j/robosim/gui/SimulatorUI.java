package br.com.r4j.robosim.gui;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.HashMap;
import java.util.Iterator;

import javax.swing.JComponent;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputAdapter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.robosim.EstimatorRendererInfo;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 *
 */
public class SimulatorUI implements BlockUI
{
	private static Log log = LogFactory.getLog(SimulatorUI.class.getName());

	private JComponent contentPane = null;
	private JComponent glassPane = null;
	private BlockUI worldUI = null;
	private BlockUI playerUI = null;
	

	protected GUIStyleManager styleManager = null;

	protected boolean bShowPlayer = false;


	public SimulatorUI()
	{
		log.debug("SimulatorUI");
		
//		slider = new JSlider(JSlider.HORIZONTAL, 0, 100, 0);
//		slider.setPaintTicks(true); slider.setPaintTicks(true); slider.setPaintLabels(true); 

		this.initActions();
	}


	public GUIStyleManager getStyleManager()	{return styleManager;}
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	public void setWorldUI(BlockUI worldUI)
	{
		this.worldUI = worldUI;
	}


	public void setplayerUI(BlockUI playerUI)
	{
		this.playerUI = playerUI;
	}


	public void showPlayer(boolean val)
	{
		bShowPlayer = val;
	}
	

	private void initActions()
	{
	}


	public void setGlassPane(JComponent glassPane)	{this.glassPane = glassPane;}
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

		this.getContentPane().removeAll();
		if (bShowPlayer)
		{
			this.getContentPane().setLayout(new BorderLayout());
			this.getContentPane().add(worldUI.getContentPane(), BorderLayout.CENTER);
			this.getContentPane().add(playerUI.getContentPane(), BorderLayout.SOUTH);
		}
		else
		{
			this.getContentPane().setLayout(new BorderLayout());
			this.getContentPane().add(worldUI.getContentPane(), BorderLayout.CENTER);
		}

		log.debug("build:end");
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
		worldUI.rebuild();
		playerUI.rebuild();
		this.build();
	}
}




