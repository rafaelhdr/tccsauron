package br.com.r4j.robosim.gui;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BoxLayout;
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
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.RobotPlayerListener;
import br.com.r4j.robosim.gui.actions.FileAction;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
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

	private ActionFileRobotPlayer robotPlayer = null;

	protected GUIStyleManager styleManager = null;


	public ActionPlayerControlUI()
	{
		log.debug("ActionPlayerControlUI");

		slider = new JSlider(JSlider.HORIZONTAL, 0, 100, 0);
		slider.setPaintTicks(true); slider.setPaintTicks(true); slider.setPaintLabels(true); 

		this.initActions();
	}


	public GUIStyleManager getStyleManager()	{return styleManager;}
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	private void initActions()
	{
		log.debug("initActions");

		actOpenActionFile = new FileAction(GUIUtil.getButtonIcon("/br/com/r4j/gui", "act16.gif", "Play", this.getClass()), "Trocar Arquivo de Ações", this, "setActionFile");
		actPlay = new PlayAction();
		actPause = new PauseAction();
		actStep = new StepAction();
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

		gbc.gridx = 1; gbc.gridy = 3;
		gb.setConstraints(btnOpenActionFile, gbc);
		comp.add(btnOpenActionFile);

		log.debug("build:end");
	}


	public void setPlayer(ActionFileRobotPlayer robotPlayer)
	{
		if (this.robotPlayer != null)
			this.robotPlayer.removeRobotPlayerListener(this);

		this.robotPlayer = robotPlayer;
		this.robotPlayer.resetPlayer();
		this.robotPlayer.addRobotPlayerListener(this);
		this.resetSlider();
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


	public void actionStarted(RobotPlayerEvent e)
	{
	}


	public void actionCompleted(RobotPlayerEvent e)
	{
		slider.getModel().setValue(e.getStepIdx());
//		slider.revalidate();
	}


	public void endOfActions(RobotPlayerEvent e)
	{
//		slider.getModel().setValue(0);
	}


	public void beginOfActions(RobotPlayerEvent e)
	{
	}


	public void actionsUpdated(RobotPlayerEvent e)
	{
		this.resetSlider();
	}


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


	public void stateChanged(ChangeEvent e)
	{
		if (e.getSource() instanceof JSlider)
		{
			JSlider source = (JSlider) e.getSource();
			robotPlayer.seek(source.getValue());
		}
	}

	
	public void setActionFile(String strFile) throws IOException
	{
		robotPlayer.setActionFile(new File(strFile));
	}


	class PlayAction extends AbstractAction
	{
		public PlayAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "play16.gif", "Play", PlayAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Play");
		}


		public void actionPerformed(ActionEvent e)
		{
			slider.removeChangeListener(ActionPlayerControlUI.this);
			slider.setEnabled(false);
			robotPlayer.play(slider);
		}
	}


	class PauseAction extends AbstractAction
	{
		public PauseAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "pause16.gif", "Pause", PauseAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Pause");
		}


		public void actionPerformed(ActionEvent e)
		{
			robotPlayer.pause();
			slider.removeChangeListener(ActionPlayerControlUI.this);
			slider.addChangeListener(ActionPlayerControlUI.this);
			slider.setEnabled(true);
		}
	}


	class StepAction extends AbstractAction
	{
		public StepAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "step16.gif", "Step", StepAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Step");
		}


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
	 */
	public void rebuild()
	{
		JComponent comp = this.getContentPane();
		comp.removeAll();
		this.build(); 
	}
}
