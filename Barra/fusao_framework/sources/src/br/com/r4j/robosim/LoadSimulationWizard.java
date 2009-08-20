package br.com.r4j.robosim;

import java.lang.reflect.*;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTextField;
import javax.swing.Spring;
import javax.swing.SpringLayout;
import javax.swing.SwingConstants;
import javax.swing.border.Border;
import javax.swing.border.EmptyBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.EventListenerList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.ObjectHolder;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.gui.DialogRendererHelper;
import br.com.r4j.gui.FileChooserWindow;
import br.com.r4j.gui.GUIUtil;
import br.com.r4j.robosim.estimator.*;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.provider.DynamicModelInfo;
import br.com.r4j.robosim.estimator.provider.EstimationComponentsInfo;
import br.com.r4j.robosim.estimator.provider.EstimationObjectsProvider;
import br.com.r4j.robosim.estimator.provider.EstimatorInfo;
import br.com.r4j.robosim.estimator.provider.SensorInfo;
import br.com.r4j.robosim.estimator.provider.SensorModelInfo;
import br.com.r4j.sun.SpringUtilities;


public class LoadSimulationWizard
{
	private static Log log = LogFactory.getLog(LoadSimulationWizard.class.getName());

	private EventListenerList listenerList = null;
	private Frame frParent = null;

	private ArrayList listUsedSensors = null;
	private RobotPlayer robotPlayer = null;
	private WorldMap worldMap = null;

	private EstimationObjectsProvider estProvider = null;


	public LoadSimulationWizard()
	{
		listenerList = new EventListenerList();

		holderFileRealReadings = new ObjectHolder();
		holderFileRealPose = new ObjectHolder();
		holderFileWorldMap = new ObjectHolder();
		holderFileActions = new ObjectHolder();
	}


	public void setProvider(EstimationObjectsProvider estProvider)
	{
		this.estProvider = estProvider;
	}

	
	public void addConfiguratorListener(ConfiguratorListener configListener)
		{listenerList.add(ConfiguratorListener.class, configListener);}
	public void removeConfiguratorListener(ConfiguratorListener configListener)
		{listenerList.remove(ConfiguratorListener.class, configListener);}

	protected void fireConfigurationDone(ConfiguratorEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for(int i = listeners.length - 2; i >= 0; i -= 2)
			if(listeners[i] == ConfiguratorListener.class)
				((ConfiguratorListener) listeners[i+1]).configurationDone(e);
	}

	
	public void setParentFrame(Frame frParent)
	{
		this.frParent = frParent;
	}


	class CommitAction extends AbstractAction
	{
		private JDialog diagBase;
		private JList listSelectedEstimators;
		public CommitAction(JDialog diagBase, JList listSelectedEstimators)
		{
			super("Ok"); this.diagBase = diagBase; this.listSelectedEstimators = listSelectedEstimators;
		}
		public void actionPerformed(ActionEvent e)
		{
			DefaultComboBoxModel modelList = (DefaultComboBoxModel) listSelectedEstimators.getModel();
			if (modelList.getSize() == 0)
			{
				JOptionPane.showMessageDialog(null, 
						"Selecione ao menos um estimador",
						"Selecione ao menos um estimador",
						JOptionPane.ERROR_MESSAGE);
				return;
			}

			listSelectedSensorInfos = new ArrayList();
			listSelectedSensorModelsInfos = new ArrayList();
			listSelectedEstimationComponentsInfos = new ArrayList();

			for (int i = 0; i < modelList.getSize(); i++)
			{
				EstimationComponentsInfo selectedEstimator = (EstimationComponentsInfo) modelList.getElementAt(i);
				listSelectedEstimationComponentsInfos.add(selectedEstimator);
				Iterator itSensorModels = selectedEstimator.getSensorModels().iterator();
				while (itSensorModels.hasNext())
				{
					SensorModelInfo mdl = (SensorModelInfo) itSensorModels.next();
					if (!listSelectedSensorModelsInfos.contains(mdl))
					{
						listSelectedSensorModelsInfos.add(mdl);
						SensorInfo sns = mdl.getSensorInfo();
						if (!listSelectedSensorInfos.contains(sns))
							listSelectedSensorInfos.add(sns);
					}
				}
			}

			diagBase.setVisible(false);
		}
	}
	class OkAction extends AbstractAction
	{
		private JDialog diagBase;
		public OkAction(JDialog diagBase)
		{
			super("Ok"); this.diagBase = diagBase;
		}
		public void actionPerformed(ActionEvent e)
		{
			diagBase.setVisible(false);
		}
	}
	class CancelAction extends AbstractAction
	{
		private JDialog diagBase;
		public CancelAction(JDialog diagBase)
		{
			super("Cancelar"); this.diagBase = diagBase;
		}
		public void actionPerformed(ActionEvent e)
		{
			diagBase.setVisible(false);
		}
	}

	private boolean bSimulationEnv = true;
	private boolean bSimulationEnvLastOk = true;
	private JRadioButton radioReal = null;
	private JRadioButton radioSimualdo = null;


	private void showEnvironmentSelection(JDialog diagBase)
	{
		JLabel lblTitle = new JLabel("Tipo de Ambiente", SwingConstants.CENTER);

		JPanel pnlContentPane = new JPanel();
		JPanel pnlRadio = new JPanel();
		JPanel pnlButtons = new JPanel();
//*		
		pnlContentPane.setLayout(new BorderLayout());
		pnlContentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
		pnlContentPane.add(lblTitle, BorderLayout.NORTH);
		lblTitle.setBorder(new EmptyBorder(0, 0, 5, 0));
		pnlContentPane.add(pnlRadio, BorderLayout.CENTER);
		pnlContentPane.add(pnlButtons, BorderLayout.SOUTH);
		pnlButtons.setBorder(new EmptyBorder(5, 0, 0, 0));
/*/
		pnlContentPane.setLayout(new BoxLayout(pnlContentPane, BoxLayout.Y_AXIS));
		pnlContentPane.add(lblTitle);
		pnlContentPane.add(pnlRadio);
		pnlContentPane.add(pnlButtons);
//*/

		diagBase.setContentPane(pnlContentPane);
//		diagBase.setContentPane(pnlButtons);

		Border etchedCanvasControls = BorderFactory.createEtchedBorder();
		pnlRadio.setBorder(etchedCanvasControls);
		pnlRadio.setLayout(new BoxLayout(pnlRadio, BoxLayout.Y_AXIS));
		ButtonGroup groupRadio = new ButtonGroup();
		ChoiceEnvironmentSelection itemList = new ChoiceEnvironmentSelection();
		radioReal = new JRadioButton("Real");
		radioSimualdo = new JRadioButton("Simulado");

		pnlRadio.add(radioSimualdo);
		groupRadio.add(radioSimualdo);
		radioSimualdo.addItemListener(itemList);
		pnlRadio.add(radioReal);
		groupRadio.add(radioReal);
		radioReal.addItemListener(itemList);

//*
		JButton btnNext = new JButton(new NextEnvironmentSelection(diagBase));
		JButton btnCancel = new JButton(new CancelAction(diagBase));
/*/
		JButton btnNext = new JButton("next");
		JButton btnCancel = new JButton("prev");
//*/		
//		btnCancel.setMinimumSize(btnCancel.getPreferredSize());
//		btnNext.setMinimumSize(btnNext.getPreferredSize());

/*		
        SpringLayout layout = new SpringLayout();
		pnlButtons.setMinimumSize(btnCancel.getMaximumSize());
        pnlButtons.setLayout(layout);
		pnlButtons.add(btnNext);
		pnlButtons.add(btnCancel);
		layout.minimumLayoutSize(pnlButtons);
		SpringLayout.Constraints constrNext = layout.getConstraints(pnlButtons);
		constrNext.setHeight(Spring.constant(30));
		layout.putConstraint(SpringLayout.WEST, btnNext, 5, SpringLayout.WEST, pnlButtons);
		layout.putConstraint(SpringLayout.NORTH, btnNext, 5, SpringLayout.NORTH, pnlButtons);
		layout.putConstraint(SpringLayout.WEST, btnCancel, 5, SpringLayout.EAST, btnNext);
		layout.putConstraint(SpringLayout.NORTH, btnCancel, 5, SpringLayout.NORTH, pnlButtons);
		//layout.putConstraint(SpringLayout.NORTH, btnCancel, 25, SpringLayout.SOUTH, pnlButtons);
		//layout.putConstraint(SpringLayout.SOUTH, btnCancel, 5, SpringLayout.SOUTH, pnlButtons);
/*/
		pnlButtons.setLayout(new BoxLayout(pnlButtons, BoxLayout.X_AXIS));
		pnlButtons.add(btnCancel);
		pnlButtons.add(btnNext);
//*/

		diagBase.pack();
	}

	class ChoiceEnvironmentSelection implements ItemListener
	{
		public void itemStateChanged(ItemEvent e)
		{
			if(e.getSource() == radioReal)
				bSimulationEnv = false;
			else
				bSimulationEnv = true;

			if (bSimulationEnv != bSimulationEnvLastOk)
			{
			}
			bSimulationEnvLastOk = bSimulationEnv;
		}
	}

	class NextEnvironmentSelection extends AbstractAction
	{
		private JDialog diagBase;
		public NextEnvironmentSelection(JDialog diagBase)
		{
			super("Ok >>"); this.diagBase = diagBase;
		}
		public void actionPerformed(ActionEvent e)
		{
			showParameterSelection(diagBase);
		}
	}


	public void setFileRealReadings(File fl)	{this.holderFileRealReadings.set(fl);}
	private ObjectHolder holderFileRealReadings = null;

	public void setFileRealPose(File fl)	{this.holderFileRealPose.set(fl);}
	private ObjectHolder holderFileRealPose = null;

	public void setFileWorldMap(File fl)	{this.holderFileWorldMap.set(fl);}
	private ObjectHolder holderFileWorldMap = null;

	private boolean bInteractive = false;

	public void setFileActions(File fl)	{this.holderFileActions.set(fl);}
	private ObjectHolder holderFileActions = null;

	private void showParameterSelection(JDialog diagBase)
	{
		try
		{
			DefaultComboBoxModel listModelSelectedEstimators = new DefaultComboBoxModel();
			JList listSelectedEstimators = new JList(listModelSelectedEstimators);
			JButton btnAdd = new JButton(new AddEstimator(diagBase, listSelectedEstimators));
			JButton btnRemove = new JButton(new RemoveEstimator(diagBase, listSelectedEstimators));

			listSelectedEstimators.setVisibleRowCount(4);

			JPanel pnlTop = new JPanel();
			GridBagLayout gb = new GridBagLayout(); 
			GridBagConstraints gbc = new GridBagConstraints();
			gbc.weightx = 0; gbc.weighty = 0;
			gbc.fill = GridBagConstraints.NONE;
			gbc.gridx = 0; gbc.gridy = 0;
			gbc.gridwidth = 1; gbc.gridheight = 3;
			gbc.insets = new Insets(1, 1, 1, 1);
			pnlTop.setLayout(gb);
			gb.setConstraints(listSelectedEstimators, gbc); pnlTop.add(listSelectedEstimators);

			gbc.gridwidth = 1; gbc.gridheight = 1;
			gbc.gridx = 1; gbc.gridy = 0;
			gb.setConstraints(btnAdd, gbc); pnlTop.add(btnAdd);
			gbc.gridx = 1; gbc.gridy = 1;
			gb.setConstraints(btnRemove, gbc); pnlTop.add(btnRemove);
			gbc.gridx = 1; gbc.gridy = 2;
			{
				JPanel pnlTmp = new JPanel();
				gb.setConstraints(pnlTmp, gbc); pnlTop.add(pnlTmp);
			}
			Border etchedTop = BorderFactory.createEtchedBorder();
			TitledBorder titledetchedTop = BorderFactory.createTitledBorder(etchedTop, "Estimadores");
			titledetchedTop.setTitleJustification(TitledBorder.RIGHT);
			titledetchedTop.setTitlePosition(TitledBorder.DEFAULT_POSITION);
			pnlTop.setBorder(titledetchedTop);

			JPanel pnlMidle = new JPanel();
			JPanel pnlContentPaneInt = new JPanel();
			pnlContentPaneInt.setLayout(new BoxLayout(pnlContentPaneInt, BoxLayout.Y_AXIS));
			pnlContentPaneInt.add(pnlTop);
			pnlContentPaneInt.add(pnlMidle);

			JLabel lblWorldMap = new JLabel();
			lblWorldMap.setText("Mapa");
			JTextField txtWorldMap = new JTextField();
			txtWorldMap.setColumns(15);
			if(holderFileWorldMap.get() != null)
				txtWorldMap.setText(((File) holderFileWorldMap.get()).getCanonicalPath());
			JButton btnFileWorldMap = new JButton(new FilePathChooser(diagBase, txtWorldMap, holderFileWorldMap));
			if(!bSimulationEnv)
			{
				JLabel lblFldRealReadings = new JLabel();
				lblFldRealReadings.setText("Leituras");
				JTextField txtFldRealReadings = new JTextField();
				txtFldRealReadings.setColumns(15);
				if(holderFileRealReadings.get() != null)
					txtFldRealReadings.setText(((File) holderFileRealReadings.get()).getCanonicalPath());
				JButton btnFileRealReadings = new JButton(new FilePathChooser(diagBase, txtFldRealReadings, holderFileRealReadings));

				JLabel lblRealPose = new JLabel();
				lblRealPose.setText("Postura Real");
				JTextField txtFldRealPose = new JTextField();
				txtFldRealPose.setColumns(15);
				if(holderFileRealPose.get() != null)
					txtFldRealPose.setText(((File) holderFileRealPose.get()).getCanonicalPath());
				JButton btnFileRealPose = new JButton(new FilePathChooser(diagBase, txtFldRealPose, holderFileRealPose));

				pnlMidle.setLayout(new SpringLayout());
				pnlMidle.add(lblFldRealReadings);
				pnlMidle.add(txtFldRealReadings);
				pnlMidle.add(btnFileRealReadings);
				pnlMidle.add(lblRealPose);
				pnlMidle.add(txtFldRealPose);
				pnlMidle.add(btnFileRealPose);
				pnlMidle.add(lblWorldMap);
				pnlMidle.add(txtWorldMap);
				pnlMidle.add(btnFileWorldMap);
				SpringUtilities.makeCompactGrid(pnlMidle, 3, 3,
												3, 3,  //initX, initY
												3, 3); //xPad, yPad
			}
			else
			{
				JLabel lblActions = new JLabel();
				lblActions.setText("A��es");
				JTextField txtFldActions = new JTextField();
				txtFldActions.setColumns(15);
				if(holderFileActions.get() != null)
					txtFldActions.setText(((File) holderFileActions.get()).getCanonicalPath());
				JButton btnFileActions = new JButton(new FilePathChooser(diagBase, txtFldActions, holderFileActions));

				pnlMidle.setLayout(new SpringLayout());
				pnlMidle.add(lblActions);
				pnlMidle.add(txtFldActions);
				pnlMidle.add(btnFileActions);
				pnlMidle.add(lblWorldMap);
				pnlMidle.add(txtWorldMap);
				pnlMidle.add(btnFileWorldMap);
				SpringUtilities.makeCompactGrid(pnlMidle, 2, 3,
												3, 3,  //initX, initY
												3, 3); //xPad, yPad
			}
			Border etchedMidle = BorderFactory.createEtchedBorder();
			TitledBorder titledetchedMidle = BorderFactory.createTitledBorder(etchedMidle, "Arquivos");
			titledetchedMidle.setTitleJustification(TitledBorder.RIGHT);
			titledetchedMidle.setTitlePosition(TitledBorder.DEFAULT_POSITION);
			pnlMidle.setBorder(titledetchedMidle);

			JButton btnPrev = new JButton(new BackParameterSelection(diagBase));
			JButton btnOk = new JButton(new CommitAction(diagBase, listSelectedEstimators));
			JPanel pnlButtons = new JPanel();
/*			
			SpringLayout layout = new SpringLayout();
			pnlButtons.setLayout(layout);
			pnlButtons.add(btnPrev);
			pnlButtons.add(btnOk);
			layout.putConstraint(SpringLayout.EAST, btnPrev, 5, SpringLayout.WEST, btnOk);
			layout.putConstraint(SpringLayout.EAST, btnOk, 5, SpringLayout.EAST, pnlButtons);
/*/
			pnlButtons.setLayout(new BoxLayout(pnlButtons, BoxLayout.X_AXIS));
			pnlButtons.add(btnPrev);
			pnlButtons.add(btnOk);
//*/

			JPanel pnlContentPane = new JPanel();
			pnlContentPane.setLayout(new BorderLayout());
			pnlContentPane.add(pnlContentPaneInt, BorderLayout.NORTH);
			pnlContentPane.add(pnlButtons, BorderLayout.SOUTH);

			diagBase.setContentPane(pnlContentPane);
			diagBase.pack();
		}
		catch (IOException e)
		{
			log.error("erro", e);
			JOptionPane.showMessageDialog(null, 
					"N�o foi poss�vel abrir o arquivo selecionado",
					"N�o foi poss�vel abrir o arquivo selecionado",
					JOptionPane.ERROR_MESSAGE);
		}
	}

	private HashMap mapCountEst = new HashMap();
	class AddEstimator extends AbstractAction
	{
		private JDialog diagBase;
		private JList listSelectedEstimators;
		public AddEstimator(JDialog diagBase, JList listSelectedEstimators)
		{
			super("Adicionar"); this.diagBase = diagBase; this.listSelectedEstimators = listSelectedEstimators;
		}
		public void actionPerformed(ActionEvent e)
		{
			JDialog diagBaseChild = DialogRendererHelper.getDialog(diagBase);
			showEstimatorSelection(diagBaseChild);
			GUIUtil.getDefaultInstance().centerDialog(diagBaseChild);
			selectedEstimator = null;
			diagBaseChild.show();
			diagBaseChild.setModal(true);
			if(selectedEstimator != null)
			{
				int count = 1;
				if (mapCountEst.containsKey(selectedEstimator))
				{
					count = ((Integer) mapCountEst.get(selectedEstimator)).intValue() + 1;
					selectedEstimator = selectedEstimator.getCopy(selectedEstimator.getName() + "_" + count);
				}
				mapCountEst.put(selectedEstimator, new Integer(count));
				DefaultComboBoxModel listModel = (DefaultComboBoxModel) listSelectedEstimators.getModel();
				listModel.addElement(selectedEstimator);
			}
		}
	}
	class RemoveEstimator extends AbstractAction
	{
		private JDialog diagBase;
		private JList listSelectedEstimators;
		public RemoveEstimator(JDialog diagBase, JList listSelectedEstimators)
		{
			super("Remover"); this.diagBase = diagBase; this.listSelectedEstimators = listSelectedEstimators;
		}
		public void actionPerformed(ActionEvent e)
		{
			int idxSel = listSelectedEstimators.getSelectedIndex();
			if(idxSel == -1)
			{
				JOptionPane.showMessageDialog(null, 
						"Selecione algum estimador para remo��o",
						"Selecione algum estimador para remo��o",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			((DefaultComboBoxModel) listSelectedEstimators.getModel()).removeElementAt(idxSel);
		}
	}
	class BackParameterSelection extends AbstractAction
	{
		private JDialog diagBase;
		public BackParameterSelection(JDialog diagBase)
		{
			super("<< Voltar"); this.diagBase = diagBase;
		}

		public void actionPerformed(ActionEvent e)
		{
			showEnvironmentSelection(diagBase);
		}
	}
	class FilePathChooser extends AbstractAction
	{
		private JTextField txtFld;
		private JDialog diagBase;
		private ObjectHolder holderFile;
		public FilePathChooser(JDialog diagBase, JTextField txtFld, ObjectHolder holderFile)
		{
			super("..."); this.txtFld = txtFld; this.diagBase = diagBase; this.holderFile = holderFile;
		}

		public void actionPerformed(ActionEvent evt)
		{
			FileChooserWindow chooser = new FileChooserWindow(txtFld.getText(), FileChooserWindow.abrir, diagBase, false);
			String strFileName = chooser.getFileName();
			if(strFileName != null && !strFileName.equals(""))
			{
				txtFld.setText(strFileName);
				holderFile.set(new File(strFileName));
			}
		}
	}


	private EstimatorInfo selectedEstimatorPhase1 = null;

	private void showEstimatorSelection(JDialog diagBase)
	{
		DefaultComboBoxModel listModelEstimators = new DefaultComboBoxModel();
		JList listEstimators = new JList(listModelEstimators);

		Iterator itEsts = estProvider.getEstimatorsInfo().iterator();
		while(itEsts.hasNext())
		{
			EstimatorInfo estInf = (EstimatorInfo) itEsts.next();
			listModelEstimators.addElement(estInf);
		}
		listEstimators.setVisibleRowCount(4);

		Border etchedCanvasControls = BorderFactory.createEtchedBorder();
		JButton btnNext = new JButton(new NextEstimatorSelection(diagBase, listEstimators));
		JButton btnCancel = new JButton(new CancelAction(diagBase));

		JPanel pnlButtons = new JPanel();
		pnlButtons.setBorder(etchedCanvasControls);
/*		
        SpringLayout layout = new SpringLayout();
        pnlButtons.setLayout(layout);
		pnlButtons.add(btnNext);
		pnlButtons.add(btnCancel);
		layout.putConstraint(SpringLayout.EAST, btnNext, 5, SpringLayout.WEST, btnCancel);
		layout.putConstraint(SpringLayout.EAST, btnCancel, 5, SpringLayout.EAST, pnlButtons);
/*/
		pnlButtons.setLayout(new BoxLayout(pnlButtons, BoxLayout.X_AXIS));
		pnlButtons.add(btnNext);
		pnlButtons.add(btnCancel);
//*/

		JPanel pnlContentPane = new JPanel();
		pnlContentPane.setLayout(new BorderLayout());
		pnlContentPane.add(listEstimators, BorderLayout.CENTER);
		pnlContentPane.add(pnlButtons, BorderLayout.SOUTH);

		diagBase.setContentPane(pnlContentPane);
		diagBase.pack();
	}

	class NextEstimatorSelection extends AbstractAction
	{
		private JDialog diagBase;
		private JList listEstimators;
		public NextEstimatorSelection(JDialog diagBase, JList listEstimators)
		{
			super("Ok >>"); this.diagBase = diagBase; this.listEstimators = listEstimators;
		}
		public void actionPerformed(ActionEvent e)
		{
			int idxSel = listEstimators.getSelectedIndex();
			if(idxSel == -1)
			{
				JOptionPane.showMessageDialog(null, 
						"Selecione algum estimador",
						"Selecione algum estimador",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			selectedEstimatorPhase1 = (EstimatorInfo) ((DefaultComboBoxModel) listEstimators.getModel()).getElementAt(idxSel);

			if (selectedEstimatorPhase1.getMaxSensors() == 0)
			{
				selectedEstimator = new EstimationComponentsInfo(selectedEstimatorPhase1.getName());
				selectedEstimator.setEstimator(selectedEstimatorPhase1);
				selectedEstimator.setDynamicModel(estProvider.getDynamicModelInfo(bSimulationEnv).getCopy());
				diagBase.setVisible(false);
			}
			else
				showSensorSelection(diagBase);
		}
	}


	private EstimationComponentsInfo selectedEstimator = null;

	private void showSensorSelection(JDialog diagBase)
	{
		DefaultComboBoxModel listModelAvailableSensors = new DefaultComboBoxModel();
		JList listAvailableSensors = new JList(listModelAvailableSensors);

		ModelFilter mdlFilter = null;
		try
		{
			Class clsEst = selectedEstimatorPhase1.getEstimatorClass();
			Method methMe = null;
			Class [] aCls = new Class[0];
			Object [] aObjParam = new Object[0];
			methMe = clsEst.getMethod("getModelFilter", aCls);
			mdlFilter = (ModelFilter) methMe.invoke(null, aObjParam);
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}

		Iterator itSens = estProvider.getSensorModelsInfo(bSimulationEnv).iterator();
		while(itSens.hasNext())
		{
			SensorModelInfo sensInf = (SensorModelInfo) itSens.next();
			if(selectedEstimatorPhase1.usesInvertedModel() && !sensInf.supportInvertedModel())
				continue;
			listModelAvailableSensors.addElement(sensInf);
		}

		DefaultComboBoxModel listModelSelectedSensors = new DefaultComboBoxModel();
		JList listSelectedSensors = new JList(listModelSelectedSensors);

		JButton btnAdd = GUIUtil.createToolBarLikeButton(new AddSensor(diagBase, listSelectedSensors, listAvailableSensors));
		JButton btnRemove = GUIUtil.createToolBarLikeButton(new RemoveSensor(diagBase, listSelectedSensors, listAvailableSensors));

		JPanel pnlTop = new JPanel();
//		pnlTop.setBorder(new EmptyBorder(5, 5, 5, 5));
		GridBagLayout gb = new GridBagLayout(); 
		GridBagConstraints gbc = new GridBagConstraints();
		pnlTop.setLayout(gb);

		gbc.weightx = 0; gbc.weighty = 0;
		gbc.fill = GridBagConstraints.NONE;
		gbc.insets = new Insets(1, 1, 1, 1);
		gbc.gridwidth = 1; gbc.gridheight = 1;
		gbc.gridx = 0; gbc.gridy = 0;
		JLabel lblNotsel = new JLabel("N�o Selecionados"); 
		gb.setConstraints(lblNotsel, gbc); pnlTop.add(lblNotsel);
		gbc.gridx = 1; gbc.gridy = 0;
		{
			JPanel pnlTmp = new JPanel();
			gb.setConstraints(pnlTmp, gbc); pnlTop.add(pnlTmp);
		}
		gbc.gridx = 2; gbc.gridy = 0;
		JLabel lblSel = new JLabel("Selecionados", SwingConstants.CENTER);
		lblSel.setMinimumSize(lblNotsel.getMinimumSize()); 
		lblSel.setPreferredSize(lblNotsel.getPreferredSize()); 
		lblSel.setMaximumSize(lblNotsel.getMaximumSize()); 
		gb.setConstraints(lblSel, gbc); pnlTop.add(lblSel);
		
		gbc.gridx = 0; gbc.gridy = 1;
		gbc.gridwidth = 1; gbc.gridheight = 3;
		gbc.insets = new Insets(5, 5, 5, 5);
		gbc.anchor = GridBagConstraints.NORTHWEST;
		gb.setConstraints(listAvailableSensors, gbc); pnlTop.add(listAvailableSensors);

		gbc.anchor = GridBagConstraints.CENTER;
		gbc.gridwidth = 1; gbc.gridheight = 1;
		gbc.gridx = 1; gbc.gridy = 1;
		gbc.insets = new Insets(5, 1, 1, 1);
		gb.setConstraints(btnAdd, gbc); pnlTop.add(btnAdd);
		gbc.gridx = 1; gbc.gridy = 2;
		gbc.insets = new Insets(1, 1, 1, 1);
		gb.setConstraints(btnRemove, gbc); pnlTop.add(btnRemove);
		gbc.gridx = 1; gbc.gridy = 3;
		gbc.weightx = 0; gbc.weighty = 1;
		{
			JPanel pnlTmp = new JPanel();
			gb.setConstraints(pnlTmp, gbc); pnlTop.add(pnlTmp);
		}

		gbc.anchor = GridBagConstraints.NORTHWEST;
		gbc.weightx = 0; gbc.weighty = 0;
		gbc.gridx = 2; gbc.gridy = 1;
		gbc.gridwidth = 1; gbc.gridheight = 3;
		gbc.insets = new Insets(5, 5, 5, 5);
		gb.setConstraints(listSelectedSensors, gbc); pnlTop.add(listSelectedSensors);

		Border etchedTop = BorderFactory.createEtchedBorder();
		TitledBorder titledetchedTop = BorderFactory.createTitledBorder(etchedTop, "Sensores");
		titledetchedTop.setTitleJustification(TitledBorder.RIGHT);
        titledetchedTop.setTitlePosition(TitledBorder.DEFAULT_POSITION);
		pnlTop.setBorder(titledetchedTop);
		pnlTop.setMinimumSize(new Dimension(400, 10));

		JButton btnPrev = new JButton(new BackSensorSelection(diagBase));
		JButton btnOk = new JButton(new OkSensor(diagBase, listSelectedSensors));
		JPanel pnlButtons = new JPanel();
/*		
        SpringLayout layout = new SpringLayout();
        pnlButtons.setLayout(layout);
		pnlButtons.add(btnPrev);
		pnlButtons.add(btnOk);
		layout.putConstraint(SpringLayout.EAST, btnPrev, 5, SpringLayout.WEST, btnOk);
		layout.putConstraint(SpringLayout.EAST, btnOk, 5, SpringLayout.EAST, pnlButtons);
/*/
		pnlButtons.setLayout(new BoxLayout(pnlButtons, BoxLayout.X_AXIS));
		pnlButtons.add(btnPrev);
		pnlButtons.add(btnOk);
//*/

		JPanel pnlContentPane = new JPanel();
		pnlContentPane.setLayout(new BorderLayout());
		pnlContentPane.add(pnlTop, BorderLayout.CENTER);
		pnlContentPane.add(pnlButtons, BorderLayout.SOUTH);
		
		diagBase.setContentPane(pnlContentPane);
		diagBase.pack();
	}

	class AddSensor extends AbstractAction
	{
		private JDialog diagBase;
		private JList listSel;
		private JList listAvail;
		public AddSensor(JDialog diagBase, JList listSel, JList listAvail)
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "AddFilter24.gif", "Adicionar", AddSensor.class));
			this.putValue(Action.SHORT_DESCRIPTION, "Adicionar");
			this.diagBase = diagBase; this.listSel = listSel; this.listAvail = listAvail;
		}
		public void actionPerformed(ActionEvent e)
		{
			int countSens = ((DefaultComboBoxModel) listSel.getModel()).getSize();
			
			int [] arrayIdxSels = listAvail.getSelectedIndices();
			if (countSens + arrayIdxSels.length > selectedEstimatorPhase1.getMaxSensors() && selectedEstimatorPhase1.getMaxSensors() != -1)
			{
				JOptionPane.showMessageDialog(null, 
						"N�mero m�ximo de sensores j� selecionados",
						"N�mero m�ximo de sensores j� selecionados",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
//			int idxSel = listAvail.getSelectedIndex();
//		if(idxSel == -1)
			if(arrayIdxSels.length == 0)
			{
				JOptionPane.showMessageDialog(null, 
						"Selecione algum sensor para adi��o",
						"Selecione algum snesor para adi��o",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			Object [] oBject = listAvail.getSelectedValues();
			for (int i = 0; i < arrayIdxSels.length; i++)
			{
				((DefaultComboBoxModel) listSel.getModel()).addElement(oBject[i]);
				((DefaultComboBoxModel) listAvail.getModel()).removeElement(oBject[i]);
			}
		}
	}
	class RemoveSensor extends AbstractAction
	{
		private JDialog diagBase;
		private JList listSel;
		private JList listAvail;
		public RemoveSensor(JDialog diagBase, JList listSel, JList listAvail)
		{
			super(null, GUIUtil.getButtonIcon("/br/com/r4j/gui", "RemoveFilter24.gif", "Remover", RemoveSensor.class)); 
			this.putValue(Action.SHORT_DESCRIPTION, "Remover");
			this.diagBase = diagBase; this.listSel = listSel; this.listAvail = listAvail;
		}
		public void actionPerformed(ActionEvent e)
		{
			int idxSel = listSel.getSelectedIndex();
			if(idxSel == -1)
			{
				JOptionPane.showMessageDialog(null, 
						"Selecione algum sensor para remo��o",
						"Selecione algum snesor para remo��o",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			((DefaultComboBoxModel) listAvail.getModel()).addElement(((DefaultComboBoxModel) listSel.getModel()).getElementAt(idxSel));
			((DefaultComboBoxModel) listSel.getModel()).removeElementAt(idxSel);
		}
	}
	class OkSensor extends AbstractAction
	{
		private JDialog diagBase;
		private JList listSel;
		public OkSensor(JDialog diagBase, JList listSel)
		{
			super("Ok"); this.diagBase = diagBase; this.listSel = listSel;
		}
		public void actionPerformed(ActionEvent e)
		{
			int countSens = ((DefaultComboBoxModel) listSel.getModel()).getSize();
			if (countSens < selectedEstimatorPhase1.getMinSensors())
			{
				JOptionPane.showMessageDialog(null, 
						"N�mero m�nimo de sensores n�o atingido",
						"N�mero m�nimo de sensores n�o atingido",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			DefaultComboBoxModel modelCombo = (DefaultComboBoxModel) listSel.getModel();
			selectedEstimator = new EstimationComponentsInfo(selectedEstimatorPhase1.getName());
			selectedEstimator.setEstimator(selectedEstimatorPhase1);
			for (int i = 0; i < countSens; i++)
				selectedEstimator.addSensorModel(((SensorModelInfo) modelCombo.getElementAt(i)).getCopy());
			selectedEstimator.setDynamicModel(estProvider.getDynamicModelInfo(bSimulationEnv).getCopy());
			diagBase.setVisible(false);
		}
	}
	class BackSensorSelection extends AbstractAction
	{
		private JDialog diagBase;
		public BackSensorSelection(JDialog diagBase)
		{
			super("<< Voltar"); this.diagBase = diagBase;
		}

		public void actionPerformed(ActionEvent e)
		{
			showEstimatorSelection(diagBase);
		}
	}


	private ArrayList listSelectedSensorInfos = null;
	private ArrayList listSelectedSensorModelsInfos = null;
	private ArrayList listSelectedEstimationComponentsInfos = null;


	public void configure(WorldMap worldMap) throws ConfiguratorException
	{
		this.worldMap = worldMap;
		JDialog diagBase = DialogRendererHelper.getDialog(frParent);
		this.showEnvironmentSelection(diagBase);
		GUIUtil.getDefaultInstance().centerDialog(diagBase);
		diagBase.setModal(true);
		
		listSelectedSensorModelsInfos = null;
		listSelectedEstimationComponentsInfos = null;
		diagBase.show();
		if (listSelectedEstimationComponentsInfos == null)
			return;

		listUsedSensors = new ArrayList();

		if (holderFileWorldMap.get() != null)
		{
			try
			{
//				worldMap = new WorldMap();
				worldMap.setMapFile((File) holderFileWorldMap.get());
			}
			catch(IOException ex)
			{
				log.error("erro", ex);
				JOptionPane.showMessageDialog(null, 
						"Ocorreu um problema abrindo o arquivo de mapa",
						"Ocorreu um problema abrindo o arquivo de mapa",
						JOptionPane.ERROR_MESSAGE);
			}
		}

		if (bSimulationEnv)
		{
			if (bInteractive)
				robotPlayer = estProvider.getSimulatedInteractivePlayer();
			else
			{
				robotPlayer = estProvider.getSimulatedActionFilePlayer();
				try
				{
					((ActionFileRobotPlayer) robotPlayer).setActionFile((File) holderFileActions.get());
				}
				catch(IOException ex)
				{
					log.error("erro", ex);
					JOptionPane.showMessageDialog(null, 
							"Ocorreu um problema abrindo o arquivo de a��es",
							"Ocorreu um problema abrindo o arquivo de a��es",
							JOptionPane.ERROR_MESSAGE);
				}
			}
		}
		else
		{
			robotPlayer = estProvider.getRealPlayer();
			try
			{
				((RealRobotPlayer) robotPlayer).setRealPoseFile((File) holderFileRealPose.get());
			}
			catch(IOException ex)
			{
				log.error("erro", ex);
				JOptionPane.showMessageDialog(null, 
						"Ocorreu um problema abrindo o arquivo de posi��es reais do rob�",
						"Ocorreu um problema abrindo o arquivo de posi��es reais do rob�",
						JOptionPane.ERROR_MESSAGE);
			}
			try
			{
				((RealRobotPlayer) robotPlayer).setReadingsFile((File) holderFileRealReadings.get());
			}
			catch(IOException ex)
			{
				log.error("erro", ex);
				JOptionPane.showMessageDialog(null, 
						"Ocorreu um problema abrindo o arquivo de leituras sensoriais",
						"Ocorreu um problema abrindo o arquivo de leituras sensoriais",
						JOptionPane.ERROR_MESSAGE);
			}
		}
		
		if (robotPlayer instanceof MapDependent)
			((MapDependent) robotPlayer).setWorldMap(worldMap);

		DynamicModelInfo dynModelInfo = estProvider.getDynamicModelInfo(bSimulationEnv);
		DynamicModel dynModel = estProvider.getDynamicModel(dynModelInfo);

		SensorInfo snsOdo = dynModelInfo.getSensorInfo();
		if (!listSelectedSensorInfos.contains(snsOdo))
			listSelectedSensorInfos.add(snsOdo);

		Iterator itSens = listSelectedSensorInfos.iterator();
		while (itSens.hasNext())
		{
			Sensor sns = estProvider.getSensor((SensorInfo) itSens.next());
			if (sns instanceof MapDependent)
				((MapDependent) sns).setWorldMap(worldMap);
			listUsedSensors.add(sns);
			robotPlayer.addSensor(sns);
		}

		itSens = listSelectedSensorModelsInfos.iterator();
		while (itSens.hasNext())
		{
			SensorModelInfo snsModelInfo = (SensorModelInfo) itSens.next();
			SensorModel snsModel = estProvider.getSensorModel(snsModelInfo);
			if (snsModel == null)
			{
				JOptionPane.showMessageDialog(null, 
						"Ocorreu um problema gerando o modelo de sensor " + snsModelInfo.getName(),
						"Ocorreu um problema gerando o modelo de sensor " + snsModelInfo.getName(),
						JOptionPane.ERROR_MESSAGE);
			}
			else
			{
				if (snsModel instanceof MapDependent)
					((MapDependent) snsModel).setWorldMap(worldMap);
				robotPlayer.addSensorModel(snsModel);
			}
		}

		Iterator itEsts = listSelectedEstimationComponentsInfos.iterator();
		while (itEsts.hasNext())
		{
			Estimator est = estProvider.createEstimator((EstimationComponentsInfo) itEsts.next());
			robotPlayer.addEstimator(est);
		}

/*
		Iterator itSensModels = robotPlayer.getSensorModels().iterator();
		while (itSensModels.hasNext())
		{
			SensorModel snsModel = (SensorModel) itSensModels.next();
			if (snsModel instanceof MapDependent)
				((MapDependent) snsModel).setWorldMap(worldMap);
		}
//*/

		if (estProvider.getDynamicModelInfo(bSimulationEnv) instanceof MapDependent)
			((MapDependent) estProvider.getDynamicModelInfo(bSimulationEnv)).setWorldMap(worldMap);
		robotPlayer.setDynamicModel(dynModel);

		this.fireConfigurationDone(new ConfiguratorEvent(this));
	}


	public RobotPlayer getRobotPlayer()
	{
		return robotPlayer;
	}


	public WorldMap getWorldMap()
	{
		return worldMap;
	}
}


