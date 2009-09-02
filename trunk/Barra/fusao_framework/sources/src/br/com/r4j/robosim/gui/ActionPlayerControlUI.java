package br.com.r4j.robosim.gui;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.GUIUtil;
import br.com.r4j.robosim.ActionFileRobotPlayer;
import br.com.r4j.robosim.RobotPlayer;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.RobotPlayerListener;
import br.com.r4j.robosim.gui.actions.FileAction;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 */
public class ActionPlayerControlUI implements BlockUI, RobotPlayerListener, ChangeListener
{
	private static Log log = LogFactory.getLog(ActionPlayerControlUI.class.getName());

	private JComponent contentPane = null;
	private JSlider slider = null;

	private Action actOpenActionFile = null;
	private Action actPlay = null;
	private Action actPause = null;
	private Action actStep = null;

	private RobotPlayer robotPlayer = null;

	protected GUIStyleManager styleManager = null;


	public ActionPlayerControlUI()
	{
		log.debug("ActionPlayerControlUI");

		slider = new JSlider(JSlider.HORIZONTAL, 0, 100, 0);
		slider.setPaintTicks(true); slider.setPaintTicks(true); slider.setPaintLabels(true); 

		this.initActions();
	}


	/** @modelguid {B86EBCB9-3B7A-493E-8452-103588F0BB9D} */
	public GUIStyleManager getStyleManager()	{return styleManager;}
	/** @modelguid {92C0906B-58B9-425F-9112-0BDB948B21BC} */
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	/** @modelguid {02DC23DB-54B4-40AE-9D63-41E2873271BE} */
	private void initActions()
	{
		log.debug("initActions");

		actOpenActionFile = new FileAction(GUIUtil.getButtonIcon("/br/com/r4j/gui", "act16.gif", "Play", this.getClass()), "Trocar Arquivo de Ações", this, "setActionFile");
		actPlay = new PlayAction();
		actPause = new PauseAction();
		actStep = new StepAction();
	}


	/** @modelguid {DD9A92ED-AE42-4C04-BC57-59EA3E032233} */
	public void setContentPane(JComponent contentPane)	{this.contentPane = contentPane;}
	/** @modelguid {907D07B5-D2A5-4AC8-A025-1377EDBB1B42} */
	public JComponent getContentPane()
	{
		if (contentPane == null)
			contentPane = new JPanel();  
		return this.contentPane;
	}
	/** @modelguid {59DDAE4A-CB96-4D99-8A17-02CA549C8450} */
	public void setMenuBar(JMenuBar menuBar)	{}	


	/** @modelguid {314A6D5C-9C20-49D4-B235-21E8F98C81B5} */
	public void build()
	{
		log.debug("build");

		JButton btnPlay = GUIUtil.createToolBarLikeButton(actPlay, 16, 16);
		JButton btnPause = GUIUtil.createToolBarLikeButton(actPause, 16, 16);
		JButton btnStep = GUIUtil.createToolBarLikeButton(actStep, 16, 16);
		JButton btnOpenActionFile = GUIUtil.createToolBarLikeButton(actOpenActionFile, 16, 16);

		JComponent comp = this.getContentPane(); 

		GridBagLayout gb = null; GridBagConstraints gbc = null;
		gb = new GridBagLayout(); gbc = new GridBagConstraints();
		gbc.anchor = GridBagConstraints.CENTER;
		gbc.insets = new Insets(1, 1, 0, 0);
		comp.setLayout(gb);

		gbc.fill = GridBagConstraints.BOTH;
		gbc.weightx = 1; gbc.weighty = 1;
		gbc.gridx = 0; gbc.gridy = 0;
		gbc.gridheight = 4;
		gb.setConstraints(slider, gbc);
		comp.add(slider);

		gbc.fill = GridBagConstraints.NONE;
		gbc.weightx = 0; gbc.weighty = 0;
		gbc.gridx = 1; gbc.gridy = 0;
		gbc.gridheight = 1;
		gb.setConstraints(btnPlay, gbc);
		comp.add(btnPlay);

		gbc.gridx = 1; gbc.gridy = 1;
		gb.setConstraints(btnPause, gbc);
		comp.add(btnPause);

		gbc.gridx = 1; gbc.gridy = 2;
		gb.setConstraints(btnStep, gbc);
		comp.add(btnStep);

		if (robotPlayer instanceof ActionFileRobotPlayer)
		{
			gbc.gridx = 1; gbc.gridy = 3;
			gb.setConstraints(btnOpenActionFile, gbc);
			comp.add(btnOpenActionFile);
		}

		log.debug("build:end");
	}


	/** @modelguid {3448BA94-BC1D-43E7-A317-697A3869F74D} */
	public void setPlayer(RobotPlayer robotPlayer)
	{
		if (this.robotPlayer != null)
			this.robotPlayer.removeRobotPlayerListener(this);

		this.robotPlayer = robotPlayer;
		this.robotPlayer.resetPlayer();
		this.robotPlayer.addRobotPlayerListener(this);
		this.resetSlider();
	}


	/** @modelguid {570E4197-F20E-499E-9274-EAF2B83B5ED6} */
	public void adjustComponents()
	{
	}



	/** @modelguid {6E3D4E4E-712F-4769-B711-22B7476A2CC5} */
	public void stop()
	{
	}


	/** @modelguid {9FA05CF6-D55B-4492-8641-91DEC313856F} */
	public void destroy()
	{
	}


	/** @modelguid {0F077828-8A1A-4269-9662-B7A503BEB86F} */
	public void actionStarted(RobotPlayerEvent e)
	{
	}


	/** @modelguid {B1F80057-A316-402E-B8E4-2CBE71F39645} */
	public void actionCompleted(RobotPlayerEvent e)
	{
		slider.getModel().setValue(e.getStepIdx());
//		slider.revalidate();
	}


	/** @modelguid {04A3D3DE-28EC-4A94-AB09-DA0DDDA6DFF1} */
	public void endOfActions(RobotPlayerEvent e)
	{
//		slider.getModel().setValue(0);
	}


	/** @modelguid {1B180E96-69F2-484F-A533-B49305926AF9} */
	public void beginOfActions(RobotPlayerEvent e)
	{
	}


	/** @modelguid {6FA721D2-2C0C-44E2-885E-832228724713} */
	public void actionsUpdated(RobotPlayerEvent e)
	{
		this.resetSlider();
	}


	/** @modelguid {12201A56-0563-4C91-A812-D49AB97F3C97} */
	private void resetSlider()
	{
		slider.setMinimum(0);
		slider.setMaximum(robotPlayer.getNumberOfSteps());
		int dTime = robotPlayer.getNumberOfSteps();
		if (dTime > 100)
		{
			slider.setMinorTickSpacing(1);
			slider.setMajorTickSpacing((int) dTime / 10);
		}
		else if (dTime > 10)
		{
			slider.setMinorTickSpacing(1);
			slider.setMajorTickSpacing((int) dTime / 3);
		}
		else
		{
			slider.setMinorTickSpacing(1);
			slider.setMajorTickSpacing(2);
		}
		slider.setPaintTicks(true); slider.setPaintTicks(true); slider.setPaintLabels(true); 
		slider.revalidate();
		slider.removeChangeListener(ActionPlayerControlUI.this);
		slider.addChangeListener(ActionPlayerControlUI.this);
	}


	/** @modelguid {770EF60E-E10F-440D-AD3B-6121949ECDEE} */
	public void stateChanged(ChangeEvent e)
	{
		if (e.getSource() instanceof JSlider)
		{
			JSlider source = (JSlider) e.getSource();
			robotPlayer.seek(source.getValue());
		}
	}

	
	/** @modelguid {25B1823E-E80D-42EE-AFAA-A6EA225D4DBE} */
	public void setActionFile(String strFile) throws IOException
	{
		if (robotPlayer instanceof ActionFileRobotPlayer)
			((ActionFileRobotPlayer) robotPlayer).setActionFile(new File(strFile));
	}


	/** @modelguid {2496159D-9108-43AC-AE4D-8651FBB975CE} */
	class PlayAction extends AbstractAction
	{
		/** @modelguid {7C7B2C3F-073D-473B-9F6B-7A9E3EF6E450} */
		public PlayAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "play16.gif", "Play", PlayAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Play");
		}


		/** @modelguid {C6453FA8-63E2-474B-B32B-7EEA04D03CB6} */
		public void actionPerformed(ActionEvent e)
		{
			slider.removeChangeListener(ActionPlayerControlUI.this);
			slider.setEnabled(false);
			robotPlayer.play(slider);
		}
	}


	/** @modelguid {D38493F0-1B62-4AF4-8729-4C12E8D0A0A8} */
	class PauseAction extends AbstractAction
	{
		/** @modelguid {7AD2562A-71E8-417E-AE6D-EE90948C65A3} */
		public PauseAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "pause16.gif", "Pause", PauseAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Pause");
		}


		/** @modelguid {08FBCE94-7B6F-4B88-B88E-E6E80605352C} */
		public void actionPerformed(ActionEvent e)
		{
			robotPlayer.pause();
			slider.removeChangeListener(ActionPlayerControlUI.this);
			slider.addChangeListener(ActionPlayerControlUI.this);
			slider.setEnabled(true);
		}
	}


	/** @modelguid {58D53356-1D2F-4E4E-B9F9-822332E3A970} */
	class StepAction extends AbstractAction
	{
		/** @modelguid {7F551BDC-8D14-4F99-8783-E5825E09AA34} */
		public StepAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "step16.gif", "Step", StepAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Step");
		}


		/** @modelguid {806A592E-3533-4142-AE2A-32BB359CB714} */
		public void actionPerformed(ActionEvent e)
		{
			slider.setEnabled(false);
			slider.removeChangeListener(ActionPlayerControlUI.this);
			robotPlayer.step();
			slider.addChangeListener(ActionPlayerControlUI.this);
			slider.setEnabled(true);
		}
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.gui.BlockUI#rebuild()
	 * @modelguid {1B6F90E5-8688-48D0-9A54-58013C79E24A}
	 */
	public void rebuild()
	{
		JComponent comp = this.getContentPane();
		comp.removeAll();
		this.build(); 
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.gui.BlockUI#setGlassPane(javax.swing.JComponent)
	 */
	public void setGlassPane(JComponent glassPane)
	{
	}
}
