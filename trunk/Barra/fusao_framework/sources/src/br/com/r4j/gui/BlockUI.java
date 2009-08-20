package br.com.r4j.gui;

import javax.swing.JComponent;
import javax.swing.JMenuBar;


public interface BlockUI
{
	public GUIStyleManager getStyleManager();

	public void setStyleManager(GUIStyleManager aStyleMan);
	
	public void setContentPane(JComponent contentPane);

	public JComponent getContentPane();

	public void setMenuBar(JMenuBar menuBar);

	public void build();

	public void adjustComponents();

	public void stop();

	public void destroy();

	/**
	 * 
	 */
	public void rebuild();
}
