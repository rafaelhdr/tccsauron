package br.com.r4j.gui;

import javax.swing.JComponent;
import javax.swing.JMenuBar;


/** @modelguid {91EDDAA4-56A6-453D-A80A-7CC2D43E699C} */
public interface BlockUI
{
	/** @modelguid {35920E08-1C05-4C2D-B281-F1015A945C36} */
	public GUIStyleManager getStyleManager();

	/** @modelguid {618D7B77-B0A2-4E8E-A50C-4AA066931CF3} */
	public void setStyleManager(GUIStyleManager aStyleMan);
	
	/** @modelguid {8CCE314D-58C2-4EE4-A7BF-744BE461EC51} */
	public void setContentPane(JComponent contentPane);

	/** @modelguid {E4898F74-CFA2-4861-92FE-2756A07881E6} */
	public JComponent getContentPane();

	/** @modelguid {BF5F97CE-17B6-4DF7-A3BD-6211A33EC277} */
	public void setMenuBar(JMenuBar menuBar);

	/** @modelguid {422245C5-4602-48D8-B4E9-2638C44283FE} */
	public void build();

	/** @modelguid {6221A253-C4C2-436E-A42D-32F54E9C26FA} */
	public void adjustComponents();

	/** @modelguid {0B49D9E8-4241-42F4-ADE6-01485BBE7426} */
	public void stop();

	/** @modelguid {63197FA1-3F33-424B-AD84-FA1C815A02C1} */
	public void destroy();

	/**
	 * 
	 * @modelguid {8285D740-4845-4F43-A454-61B623F27F34}
	 */
	public void rebuild();

	/**
	 * @param glassPane
	 */
	public void setGlassPane(JComponent glassPane);
}
