package br.com.r4j.robosim.gui;

import java.awt.BorderLayout;

import javax.swing.JComponent;
import javax.swing.JMenuBar;
import javax.swing.JPanel;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.GUIStyleManager;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 *
 * @modelguid {EFC0A38E-98B5-4FF7-8CA0-270A1F108B84}
 */
public class SimulatorUI implements BlockUI
{
	/** @modelguid {B0B5D953-44EB-4AD3-A6D4-D352BD34AB05} */
	private static Log log = LogFactory.getLog(SimulatorUI.class.getName());

	/** @modelguid {5E4F3F0F-D8DB-4E99-A358-2CE316650AEB} */
	private JComponent contentPane = null;
	/** @modelguid {A6137551-C63F-4681-BA2D-C6E3577E3BAE} */
	private JComponent glassPane = null;
	/** @modelguid {F856EDB4-C02D-482F-BFBB-50CD3A45BCA9} */
	private BlockUI worldUI = null;
	/** @modelguid {F725ACCC-A796-442A-8F7A-D95177463A54} */
	private BlockUI playerUI = null;
	

	/** @modelguid {D6A62885-B0AE-4E61-9111-7DB824DB73A4} */
	protected GUIStyleManager styleManager = null;

	/** @modelguid {DE7051A9-2394-438C-B759-41924BC4F519} */
	protected boolean bShowPlayer = false;


	/** @modelguid {DFC3A73D-B1A6-46DE-9637-710FAB56E5C4} */
	public SimulatorUI()
	{
		log.debug("SimulatorUI");
		
//		slider = new JSlider(JSlider.HORIZONTAL, 0, 100, 0);
//		slider.setPaintTicks(true); slider.setPaintTicks(true); slider.setPaintLabels(true); 

		this.initActions();
	}


	/** @modelguid {FAC93016-3183-4452-9B9D-CA98C4FDD5B0} */
	public GUIStyleManager getStyleManager()	{return styleManager;}
	/** @modelguid {CD6635E3-94F8-4D02-AF13-53B30341CF77} */
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	/** @modelguid {B9F4A15B-26E6-4036-942F-3D432C395C3A} */
	public void setWorldUI(BlockUI worldUI)
	{
		this.worldUI = worldUI;
	}


	/** @modelguid {FD3D6C71-0225-452C-A5A4-FC90700B9AFE} */
	public void setplayerUI(BlockUI playerUI)
	{
		this.playerUI = playerUI;
	}


	/** @modelguid {39A3B22E-4E58-4D7A-9F90-1E7369015358} */
	public void showPlayer(boolean val)
	{
		bShowPlayer = val;
	}
	

	/** @modelguid {089D2B9B-5F1D-45ED-AE2A-FA80A35229FF} */
	private void initActions()
	{
	}


	/** @modelguid {C468F929-D661-4015-A5C2-554E1608ACA0} */
	public void setGlassPane(JComponent glassPane)	{this.glassPane = glassPane;}
	/** @modelguid {0D01EF64-5BDF-421E-A8D6-22E95059C907} */
	public void setContentPane(JComponent contentPane)	{this.contentPane = contentPane;}
	/** @modelguid {31512FE9-9BFC-41EE-99BC-21DC1098EFFC} */
	public JComponent getContentPane()
	{
		if (contentPane == null)
			contentPane = new JPanel();  
		return this.contentPane;
	}
	/** @modelguid {BABAB2DE-0F1C-4B22-B1AC-33BFA357DFA4} */
	public void setMenuBar(JMenuBar menuBar)	{}	


	/** @modelguid {C65F6004-0966-4DD9-B0B4-6DE31F727815} */
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


	/** @modelguid {2DB14505-74D5-4B11-BD04-A1FEE1CAE962} */
	public void adjustComponents()
	{
	}


	/** @modelguid {AF158292-4DEA-47E7-8F36-5A933BAD2C65} */
	public void stop()
	{
	}


	/** @modelguid {002DFC5F-FC15-4748-B9EE-EAA305EB6B74} */
	public void destroy()
	{
	}


	/** @modelguid {D4A09A36-0E8B-41ED-94FB-BD46FD21B3AC} */
	public void rebuild()
	{
		worldUI.rebuild();
		playerUI.rebuild();
		this.build();
	}
}




