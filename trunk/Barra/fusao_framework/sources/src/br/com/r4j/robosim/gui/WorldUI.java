package br.com.r4j.robosim.gui;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.MouseEvent;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.HashMap;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JToggleButton;
import javax.swing.JToolBar;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.MouseInputAdapter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.gui.BaseCanvas;
import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.CanvasMouseListener;
import br.com.r4j.gui.ConfigurationUI;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.GUIUtil;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.RobotPlayerListener;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.robosim.InfoSink;


/**
 *
 * Responsável por controlar a renderização do mundo do robô.
 *
 */
public class WorldUI implements BlockUI, RobotPlayerListener, ChangeListener, InfoSink
{
	private static Log log = LogFactory.getLog(WorldUI.class.getName());


	private JComponent contentPane = null;
	private BaseCanvas canvas = null;

	private JLabel lblRender = null;
	private JLabel lblNotRender = null;

	private JLabel lblInfoOne = null;
	private JLabel lblInfoTwo = null;
	private JLabel lblInfoThree = null;


	private JList lstSwingSelectedRender = null;
	private DefaultComboBoxModel listModelSelectedRender = null;

	private JList lstSwingSelectedRenderNot = null;
	private DefaultComboBoxModel listModelSelectedRenderNot = null;

	private Action actAddRender = null;
	private Action actRemoveRender = null;

	private Action actShowCross = null;
	private Action actShowRules = null;

	private Action actZoomMode = null;
	private Action actMouseMode = null;

	private JToggleButton btnZoomMode = null;
	private JToggleButton btnMouseMode = null;

	private JComponent glassPane = null;


	public final int ZOOM_MODE = 1;
	public final int MOUSE_MODE = 2;
	private int stateMouse = ZOOM_MODE;
	
	private HashMap mapList2Holder = null;

	protected GUIStyleManager styleManager = null;

	protected boolean bRendering = false;


	public WorldUI()
	{
		log.debug("WorldUI");
		
		mapList2Holder = new HashMap();

		canvas = new BaseCanvas();
		canvas.showRules(false);
		canvas.showCross(false);
		canvas.drawSelections(false);
		{
			BufferedImage buffImg = new BufferedImage(400, 400, BufferedImage.TYPE_INT_ARGB);
			Graphics2D g2d = buffImg.createGraphics();
			canvas.setBackImage(buffImg);
		}
		canvas.enableRendererListeners(true);

		listModelSelectedRender = new DefaultComboBoxModel();
		lstSwingSelectedRender = new JList(listModelSelectedRender);
		listModelSelectedRenderNot = new DefaultComboBoxModel();
		lstSwingSelectedRenderNot = new JList(listModelSelectedRenderNot);

		lblInfoOne = new JLabel("Info 1");
		lblInfoTwo = new JLabel("Info 2");
		lblInfoThree = new JLabel("Info 3");


		this.initActions();
	}


	/** @modelguid {D14B7778-D93B-4BF9-9143-21DFBBBE69D7} */
	public GUIStyleManager getStyleManager()	{return styleManager;}
	/** @modelguid {0555302C-85FF-4FE1-B41D-0B647B3FAE75} */
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	/** @modelguid {80F91B09-19A0-445A-8725-F4B5C5291365} */
	private void initActions()
	{
		log.debug("initActions");

		actAddRender = new AddRenderAction();
		actRemoveRender = new RemoveRenderAction();

		actShowCross = new ShowCrossAction(canvas);
		actShowRules = new ShowRulesAction(canvas);

		actZoomMode = new MouseModeAction("ZoomMode24.gif", "Modo Zoom", ZOOM_MODE);
		actMouseMode = new MouseModeAction("MouseMode24.gif", "Modo Livre", MOUSE_MODE);

		lstSwingSelectedRender.addMouseListener(new MouseOverPopupHandlerListener(lstSwingSelectedRender));
		lstSwingSelectedRenderNot.addMouseListener(new MouseOverPopupHandlerListener(lstSwingSelectedRenderNot));
	}

	/** @modelguid {E28FE32E-27AE-4E9A-B621-2BA2AD6D6C0F} */
	public void setGlassPane(JComponent glassPane)	{this.glassPane = glassPane;}


	/** @modelguid {B48CDCFB-D3D1-42F8-9B09-C0B87A760D2E} */
	public void setContentPane(JComponent contentPane)	{this.contentPane = contentPane;}
	/** @modelguid {344DBB5C-F4FE-4CA9-AAAF-FCDD820918A1} */
	public JComponent getContentPane()
	{
		if (contentPane == null)
			contentPane = new JPanel();  
		return contentPane;
	}
	/** @modelguid {5C20D19F-CD63-472B-B3EB-C1451D4826E6} */
	public void setMenuBar(JMenuBar menuBar)	{}	


	/** @modelguid {A7549CB8-A712-4B88-A650-6225C05D9CB8} */
	public void build()
	{
		this.getContentPane().removeAll();
		log.debug("build");

		lblRender = new JLabel();
		lblNotRender = new JLabel();

		GridBagLayout gb = null; GridBagConstraints gbc = null;
		gb = new GridBagLayout(); gbc = new GridBagConstraints();
		gbc.weightx = 0; gbc.weighty = 0;
		gbc.fill = GridBagConstraints.NONE;
		gbc.gridx = 0; gbc.gridy = 0;
		gbc.insets = new Insets(1, 1, 1, 1);
        this.getContentPane().setLayout(gb);

		int gridYCounter = 0;

		{
			gbc.weightx = 0.6; gbc.weighty = 0;
			gbc.gridx = 0; gbc.gridy = gridYCounter;
			gbc.gridwidth = 1; gbc.gridheight = 1;
			gbc.anchor = GridBagConstraints.NORTHWEST;
			JPanel pnlToolbarMain = new JPanel();
			gb.setConstraints(pnlToolbarMain, gbc); this.getContentPane().add(pnlToolbarMain);
			pnlToolbarMain.setLayout(new BorderLayout());
	        JToolBar toolBar = new JToolBar("Ferramentas Gerais");
	        pnlToolbarMain.add(toolBar, BorderLayout.PAGE_START);
			toolBar.setFloatable(false);
			toolBar.setRollover(false);

			JToggleButton btnShowCross = new JToggleButton(actShowCross);
			JToggleButton btnShowRules = new JToggleButton(actShowRules);
			toolBar.add(btnShowCross);
			toolBar.add(btnShowRules);
	        toolBar.addSeparator();
			btnZoomMode = new JToggleButton(actZoomMode);
			btnMouseMode = new JToggleButton(actMouseMode);
			toolBar.add(btnZoomMode);
			toolBar.add(btnMouseMode);
			btnMouseMode.setEnabled(true);
		}

		gbc.anchor = GridBagConstraints.NORTH;
		gbc.fill = GridBagConstraints.HORIZONTAL;
		gbc.weightx = 0.2; gbc.weighty = 0;
		gbc.gridwidth = 1; gbc.gridheight = 1;
		gbc.gridx = 1; gbc.gridy = 0;
		gb.setConstraints(lblInfoOne, gbc); this.getContentPane().add(lblInfoOne);
		gbc.gridx = 1; gbc.gridy = 1;
		gb.setConstraints(lblInfoTwo, gbc); this.getContentPane().add(lblInfoTwo);
		gbc.gridx = 1; gbc.gridy = 2;
		gb.setConstraints(lblInfoThree, gbc); this.getContentPane().add(lblInfoThree);


		gbc.anchor = GridBagConstraints.NORTH;
		gbc.fill = GridBagConstraints.HORIZONTAL;
		gbc.weightx = 0.2; gbc.weighty = 0;
		gbc.gridx = 2; gbc.gridy = gridYCounter;
		gbc.gridwidth = 1; gbc.gridheight = 3;
		JScrollPane scrlPaneSelectedRenderNot = new JScrollPane(lstSwingSelectedRenderNot);
		scrlPaneSelectedRenderNot.setMinimumSize(new Dimension(100, 80));
		scrlPaneSelectedRenderNot.setMaximumSize(new Dimension(100, 80));
		scrlPaneSelectedRenderNot.setPreferredSize(new Dimension(100, 80));
		gb.setConstraints(scrlPaneSelectedRenderNot, gbc); this.getContentPane().add(scrlPaneSelectedRenderNot);

		JButton btnAddFilter = GUIUtil.createToolBarLikeButton(actAddRender);
		gbc.weightx = 0; gbc.weighty = 0;
		gbc.gridx = 3; gbc.gridy = gridYCounter;
		gbc.gridwidth = 1; gbc.gridheight = 1;
		gb.setConstraints(btnAddFilter, gbc); this.getContentPane().add(btnAddFilter);
		JButton btnRemoveFilter = GUIUtil.createToolBarLikeButton(actRemoveRender);
		gbc.weightx = 0; gbc.weighty = 0;
		gbc.gridx = 3; gbc.gridy = gridYCounter + 1;
		gbc.gridwidth = 1; gbc.gridheight = 1;
		gb.setConstraints(btnRemoveFilter, gbc); this.getContentPane().add(btnRemoveFilter);

		gbc.weightx = 0.2; gbc.weighty = 0;
		gbc.gridx = 4; gbc.gridy = gridYCounter;
		gbc.gridwidth = 1; gbc.gridheight = 3;
		JScrollPane scrlPaneSelectedRender = new JScrollPane(lstSwingSelectedRender);
		scrlPaneSelectedRender.setMinimumSize(new Dimension(100, 80));
		scrlPaneSelectedRender.setMaximumSize(new Dimension(100, 80));
		scrlPaneSelectedRender.setPreferredSize(new Dimension(100, 80));
		gb.setConstraints(scrlPaneSelectedRender, gbc); this.getContentPane().add(scrlPaneSelectedRender);

		gridYCounter += 3;

		// area 2: botões de conexão.
		gbc.weightx = 1; gbc.weighty = 1;
		gbc.anchor = GridBagConstraints.NORTH;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.gridx = 0; gbc.gridy = gridYCounter;
		gbc.gridwidth = 5;
		gbc.gridheight = GridBagConstraints.REMAINDER;
		JScrollPane scrollCanvas = new JScrollPane(canvas);
		gb.setConstraints(scrollCanvas, gbc); this.getContentPane().add(scrollCanvas);

		canvas.addTopComponent(glassPane);

		log.debug("build:end");
	}


	public void setCanvasZoom(Rectangle2D rectBounding, int padX, int padY)
	{
		double width = 600, height = 400;
		
		double widthFactor = width/rectBounding.getWidth(), heightFactor = height/rectBounding.getWidth();

		double factor = widthFactor;		
		if (factor > heightFactor)
			factor = heightFactor;
		
		canvas.moveImage((int) -rectBounding.getMinX(), (int) -rectBounding.getMinY());
		canvas.zoom((float) factor, (float) factor);
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


	public void addRendererListener(RendererListener listener)
	{
		canvas.addRendererListener(listener);
		RendererListenerHolder hldr = new RendererListenerHolder(listener);
		listModelSelectedRender.addElement(hldr);
		mapList2Holder.put(listener, hldr);
	}


	public void removeRendererListener(RendererListener listener)
	{
		RendererListenerHolder hldr = (RendererListenerHolder) mapList2Holder.get(listener);
		if (hldr != null)
		{
			canvas.removeRendererListener(listener);
			listModelSelectedRender.removeElement(hldr);
			listModelSelectedRenderNot.removeElement(hldr);
		}
		else
			log.warn("removeRendererListener: listener não presente: " + listener.getName());
	}


	public void stateChanged(ChangeEvent e)
	{
		if (e.getSource() instanceof WorldMap)
		{
			this.erase();
			this.render();
		}
	}


	public void actionStarted(RobotPlayerEvent e)
	{
	}


	public void actionCompleted(RobotPlayerEvent e)
	{
		if (!bRendering)
		{
			bRendering = true;
			try
			{
				this.erase();
				this.render();
			}
			catch (Exception ex)
			{
				log.error("actionCompleted", ex);
			}
			finally
			{
				bRendering = false;
			}
		}
	}


	public void endOfActions(RobotPlayerEvent e)
	{
		if (!bRendering)
		{
			bRendering = true;
			try
			{
				this.erase();
				this.render();
			}
			catch (Exception ex)
			{
				log.error("endOfActions", ex);
			}
			finally
			{
				bRendering = false;
			}
		}
	}


	public void beginOfActions(RobotPlayerEvent e)
	{
		if (!bRendering)
		{
			bRendering = true;
			try
			{
				this.erase();
				this.render();
			}
			catch (Exception ex)
			{
				log.error("beginOfActions", ex);
			}
			finally
			{
				bRendering = false;
			}
		}
	}


	public void actionsUpdated(RobotPlayerEvent e)
	{
		if (!bRendering)
		{
			bRendering = true;
			try
			{
				this.erase();
				this.render();
			}
			catch (Exception ex)
			{
				log.error("actionsUpdated", ex);
			}
			finally
			{
				bRendering = false;
			}
		}
	}


	public void setInfo(int infoIdx, String strInfo)
	{
		log.debug("INFO:"+infoIdx+"->"+strInfo);
		switch (infoIdx)
		{
			case 1:
			{
				lblInfoOne.setText(strInfo);
				break;
			}
			case 2:
			{
				lblInfoTwo.setText(strInfo);
				break;
			}
			case 3:
			{
				lblInfoThree.setText(strInfo);
				break;
			}
			default:
			{
				break;
			}
		}
	}

	
	/**
	 * Métodos para avisar o mundoUI que ele precisa realizar um update.
	 *
	 */
	public void erase()
	{
		canvas.erase();
	}
	

	public void render()
	{
		canvas.render();
	}


	protected void setMouseState(int state)
	{
		switch (stateMouse)
		{
			case MOUSE_MODE:
			{
				btnMouseMode.setSelected(false);
				canvas.enableCanvasMouseListeners(false);
//				canvas.enableRendererListeners(false);
				break;
			}
			case ZOOM_MODE: default:
			{
				btnZoomMode.setSelected(false);
				canvas.enableZoom(false);
				break;
			}
		}
		stateMouse = state;
		switch (stateMouse)
		{
			case MOUSE_MODE:
			{
				btnMouseMode.setSelected(true);
				canvas.enableCanvasMouseListeners(true);
				canvas.enableRendererListeners(true);
				break;
			}
			case ZOOM_MODE: default:
			{
				btnZoomMode.setSelected(true);
				canvas.enableZoom(true);
				break;
			}
		}
	}


	// Classes para cuidar dos filtros
	class AddRenderAction extends AbstractAction
	{
		public AddRenderAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "AddFilter24.gif", "Exibir", ShowRulesAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Exibir");
		}


		public void actionPerformed(ActionEvent e)
		{
			synchronized (lstSwingSelectedRender)
			{
				if (lstSwingSelectedRenderNot.getSelectedIndex() != -1)
				{
					Object [] oBject = lstSwingSelectedRenderNot.getSelectedValues();
					int [] idxs = lstSwingSelectedRenderNot.getSelectedIndices();
					for (int i = 0; i < idxs.length; i++)
					{
						listModelSelectedRenderNot.removeElement(oBject[i]);
						listModelSelectedRender.addElement(oBject[i]);
						canvas.addRendererListener(((RendererListenerHolder) oBject[i]).getRendererListener());
					}
				}
			}
		}
	}


	class RemoveRenderAction extends AbstractAction
	{
		public RemoveRenderAction()
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "RemoveFilter24.gif", "Esconder", RemoveRenderAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Esconder");
		}


		public void actionPerformed(ActionEvent e)
		{
			synchronized (lstSwingSelectedRender)
			{
				if (lstSwingSelectedRender.getSelectedIndex() != -1)
				{
					Object [] oBject = lstSwingSelectedRender.getSelectedValues();
					int [] idxs = lstSwingSelectedRender.getSelectedIndices();
					for (int i = 0; i < idxs.length; i++)
					{
						listModelSelectedRender.removeElement(oBject[i]);
						listModelSelectedRenderNot.addElement(oBject[i]);
						canvas.removeRendererListener(((RendererListenerHolder) oBject[i]).getRendererListener());
					}
				}
			}
		}
	}


	/**
	 * Permite abrir a configuração de um Renderer na lista de filtros selecionados.
	 */
	class MouseOverPopupHandlerListener extends MouseInputAdapter
	{
		private JList listTarget = null;

		public MouseOverPopupHandlerListener(JList listTarget)
		{
			this.listTarget = listTarget;
		}
		

		public void mousePressed(MouseEvent e)		{this.mouseClicked(e);}
		public void mouseReleased(MouseEvent e)		{this.mouseClicked(e);}
		public void mouseClicked(MouseEvent e)
		{
			if (e.isPopupTrigger())
			{
				int index = listTarget.locationToIndex(e.getPoint());
				log.debug("MouseOverPopupHandlerListener::mouseClicked:index = " + index);
				if (index > -1)
				{
					Object oBject = listTarget.getModel().getElementAt(index);

					if (oBject instanceof ConfigurationUI)
					{
						if (oBject instanceof CanvasMouseListener)
							canvas.addCanvasMouseListener((CanvasMouseListener) oBject);
						((ConfigurationUI) oBject).configure(listTarget);
						if (oBject instanceof CanvasMouseListener)
							canvas.removeCanvasMouseListener((CanvasMouseListener) oBject);
					}
				}
			}
		}
	}


	class MouseModeAction extends AbstractAction
	{
		private int state = -1;

		public MouseModeAction(String strIcon, String toolTip, int state)
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", strIcon, toolTip, MouseModeAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, toolTip);
			this.state = state;
		}


		public void actionPerformed(ActionEvent e)
		{
			setMouseState(state);
		}
	}


	class ShowCrossAction extends AbstractAction
	{
		private BaseCanvas canvas = null;

		public ShowCrossAction(BaseCanvas canvas)
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "ShowCrosses24.gif", "Exibir Cruz", ShowCrossAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Exibir Cruz");
			this.canvas = canvas;
		}


		public void actionPerformed(ActionEvent e)
		{
			if (canvas != null)
				canvas.showCross(!canvas.isShowingCross());
		}
	}

		
	class ShowRulesAction extends AbstractAction
	{
		private BaseCanvas canvas = null;

		public ShowRulesAction(BaseCanvas canvas)
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "ShowRules24.gif", "Exibir Reguas", ShowRulesAction.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Exibir Reguas");
			this.canvas = canvas;
		}


		public void actionPerformed(ActionEvent e)
		{
			if (canvas != null)
				canvas.showRules(!canvas.isShowingRules());
		}
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.gui.BlockUI#rebuild()
	 * @modelguid {187E4713-8FE4-422D-9CBA-1A87D0CA4B06}
	 */
	public void rebuild()
	{
		JComponent comp = this.getContentPane();
		comp.removeAll();
		this.build(); 
	}
}
