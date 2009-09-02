package br.com.r4j.robosim.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.Icon;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputAdapter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.LogarithmicAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.StandardXYItemRenderer;
import org.jfree.data.XYSeries;
import org.jfree.data.XYSeriesCollection;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.commons.util.ObjectHolder;
import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.DialogRendererHelper;
import br.com.r4j.gui.FileChooserWindow;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.GUIUtil;
import br.com.r4j.gui.swing.JErrorMessageBox;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.NoiseControlInfo;
import br.com.r4j.robosim.StatsComponent;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.EstimationStatisticsEvent;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.EstimatorEngineListener;


/**
 *
 * Responsável por renderizar a UI de controle do player de ações.
 *
 *
 */
public class RMSStatsUI implements BlockUI, StatsComponent, EstimatorEngineListener
{
	private static Log log = LogFactory.getLog(RMSStatsUI.class.getName());

	private JComponent contentPane = null;
	private EstimatesEngine estEngine = null;
	protected GUIStyleManager styleManager = null;
	private StatsUI statsUI = null;

	private JComponent glassPane = null;

	private ChartPanel chartPanel = null;
	private JFreeChart chart = null;

	private JPanel pnlChartBase = null;
	private JPanel pnlControls = null;

	private ButtonGroup groupLogLin = null;

	private JRadioButton optLin = null;
	private JRadioButton optLog = null;
	private JButton btnRecalc = null;
	private JButton btnShow = null;

	private Legend legend = null;
	private MouseInputAdapter mouseInLegend = null;
	private Legend legendRMS = null;
	private MouseInputAdapter mouseInLegendRMS = null;
	private Legend legendTime = null;
	private MouseInputAdapter mouseInLegendTime = null;

	private boolean bHasValuesCalculated = false;

	private JTextField txtFldCountIt = null;

//	private AbstractDoubleVector [][] arrayMeans = null;
//	private AbstractDoubleSquareMatrix [][] arrayCovars = null;
	private AbstractDoubleVector [] arrayRealPoses =  null;

	// est / it / component
	private double [][][] arrayRMSMeansPerComponent = null;

	// est / it 
	private double [][] arrayRMSMeansTotal = null;

	// est
	private double [] arrayRMSMeansTotalMean = null;
	
	// est
	private long [] arrayTimes = null;

	private JTextField txtBaseFileEstimatorResultsPerIt = null;
	private JTextField txtBaseFileEstimatorMeans = null;
	private ObjectHolder holderBaseFileEstimatorResultsPerIt = null;
	private ObjectHolder holderBaseFileEstimatorMeans = null;
	private int itNumber = 10;


	
	public RMSStatsUI(StatsUI statsUI)
	{
		log.debug("RMSStatsUI");
		bHasValuesCalculated = false;
		this.statsUI = statsUI;
	
		legend = new Legend();
		legend.setTitle("Legenda");
		
		legendRMS = new Legend();
		legendRMS.setTitle("RMS médio");
		legendRMS.setBackgroundColor(new Color(191, 191, 255));
		legendRMS.setBorderColor(new Color(128, 0, 255));
		
		legendTime = new Legend();
		legendTime.setTitle("Tempo médio");
		legendTime.setBackgroundColor(new Color(0, 251, 125));
		legendTime.setBorderColor(new Color(0, 147, 73));

		holderBaseFileEstimatorResultsPerIt = new ObjectHolder();
		holderBaseFileEstimatorMeans = new ObjectHolder();
	}


	public GUIStyleManager getStyleManager()	{return styleManager;}
	public void setStyleManager(GUIStyleManager aStyleMan)	{styleManager = aStyleMan;}


	public void setContentPane(JComponent contentPane)	{this.contentPane = contentPane;}
	public JComponent getContentPane()
	{
		if (contentPane == null)
			contentPane = new JPanel();  
		return this.contentPane;
	}


	public void setGlassPane(JComponent glassPane)	{this.glassPane = glassPane;}


	public String getShortName()
	{
		return "RMS";
	}
	
	public Icon getIcon()
	{
		return null;
	}

	public String getDescription()
	{
		return "Exibe o RMS de cada estimador para cada iteração";
	}


	public void setMenuBar(JMenuBar menuBar)	{}


	public void build()
	{
		log.debug("build");
		JComponent cntPane = this.getContentPane();
		cntPane.setLayout(new BorderLayout());

		pnlChartBase = new JPanel();
		pnlControls = new JPanel();

		JDialog diagBase = DialogRendererHelper.getDialog(null);

		txtFldCountIt = new JTextField();
		txtFldCountIt.setColumns(5);
		txtFldCountIt.setText(itNumber + "");
		JLabel lblCountIt = new JLabel("Iterações");

		JLabel lblBaseFileEstimatorResultsPerIt = new JLabel();
		lblBaseFileEstimatorResultsPerIt.setText("Arquivo-base por iteração");
		txtBaseFileEstimatorResultsPerIt = new JTextField();
		txtBaseFileEstimatorResultsPerIt.setColumns(15);
		if(holderBaseFileEstimatorResultsPerIt.get() != null)
			txtBaseFileEstimatorResultsPerIt.setText((String) holderBaseFileEstimatorResultsPerIt.get());
		JButton btnBaseFileEstimatorResultsPerIt = new JButton(new FilePathChooser(diagBase, txtBaseFileEstimatorResultsPerIt, holderBaseFileEstimatorResultsPerIt));

		JLabel lblBaseFileEstimatorMeans = new JLabel();
		lblBaseFileEstimatorMeans.setText("Arquivo-base para média");
		txtBaseFileEstimatorMeans = new JTextField();
		txtBaseFileEstimatorMeans.setColumns(15);
		if(holderBaseFileEstimatorMeans.get() != null)
			txtBaseFileEstimatorMeans.setText((String) holderBaseFileEstimatorMeans.get());
		JButton btnBaseFileEstimatorMeans = new JButton(new FilePathChooser(diagBase, txtBaseFileEstimatorMeans, holderBaseFileEstimatorMeans));

		ToggleLogListener togList = new ToggleLogListener();

		optLin = new JRadioButton("Linear");
		optLin.setActionCommand("linear");
		optLin.setSelected(true);
		optLin.addActionListener(togList);

		optLog = new JRadioButton("Logarítmica");
		optLog.setActionCommand("log");
		optLog.setSelected(false);
		optLog.addActionListener(togList);

		groupLogLin = new ButtonGroup();
		groupLogLin.add(optLin);
		groupLogLin.add(optLog);

		btnRecalc = new JButton();
		btnRecalc.setText("Calcular");
		btnRecalc.addActionListener(new CalcActionListener());

		btnShow = new JButton();
		btnShow.setText("Exibir");
		btnShow.addActionListener(new ShowActionListener());

		cntPane.add(pnlChartBase, BorderLayout.CENTER);
		cntPane.add(pnlControls, BorderLayout.SOUTH);

		JPanel pnlLogLin = new JPanel();

		pnlLogLin.setLayout(new BoxLayout(pnlLogLin, BoxLayout.Y_AXIS));
		pnlLogLin.add(optLog);
		pnlLogLin.add(optLin);
		pnlLogLin.setBorder(styleManager.getTitledBorder("Escala RMS", GUIStyleManager.REGULAR));

		GridBagLayout gb = new GridBagLayout(); GridBagConstraints gbc = new GridBagConstraints();
		pnlControls.setLayout(gb);
		gbc.weighty = 0;
		gbc.insets = new Insets(1, 1, 1, 1);

		{
			JPanel pnlTmp = new JPanel(); 
			gbc.fill = GridBagConstraints.NONE;
			gbc.anchor = GridBagConstraints.CENTER;
			gbc.weightx = 0; gbc.weighty = 0;
			gbc.gridheight = 1;
			gbc.gridx = 0; gbc.gridy = 0;
			gb.setConstraints(pnlTmp, gbc); pnlControls.add(pnlTmp);
		}

		gbc.fill = GridBagConstraints.HORIZONTAL;
		gbc.anchor = GridBagConstraints.WEST;
		gbc.weightx = 0; gbc.weighty = 0;
		gbc.gridheight = 1;
		
		gbc.gridx = 1; gbc.gridy = 0;
		gb.setConstraints(lblCountIt, gbc); pnlControls.add(lblCountIt);
		gbc.gridx = 1; gbc.gridy = 1;
		gb.setConstraints(txtFldCountIt, gbc); pnlControls.add(txtFldCountIt);

		gbc.gridx = 2; gbc.gridy = 0;
		gbc.gridwidth = 2;
		gbc.anchor = GridBagConstraints.CENTER;
		gbc.fill = GridBagConstraints.NONE;
		gb.setConstraints(lblBaseFileEstimatorResultsPerIt, gbc); pnlControls.add(lblBaseFileEstimatorResultsPerIt);
		gbc.gridwidth = 1;
		gbc.gridx = 2; gbc.gridy = 1;
		gbc.weightx = 0.5;
		gbc.anchor = GridBagConstraints.EAST;
		gbc.fill = GridBagConstraints.HORIZONTAL;
		gb.setConstraints(txtBaseFileEstimatorResultsPerIt, gbc); pnlControls.add(txtBaseFileEstimatorResultsPerIt);
		gbc.gridx = 3; gbc.gridy = 1;
		gbc.weightx = 0;
		gbc.fill = GridBagConstraints.NONE;
		gb.setConstraints(btnBaseFileEstimatorResultsPerIt, gbc); pnlControls.add(btnBaseFileEstimatorResultsPerIt);

		gbc.gridx = 4; gbc.gridy = 0;
		gbc.gridwidth = 2;
		gbc.anchor = GridBagConstraints.CENTER;
		gbc.fill = GridBagConstraints.NONE;
		gb.setConstraints(lblBaseFileEstimatorMeans, gbc); pnlControls.add(lblBaseFileEstimatorMeans);
		gbc.gridwidth = 1;
		gbc.gridx = 4; gbc.gridy = 1;
		gbc.weightx = 0.5;
		gbc.anchor = GridBagConstraints.EAST;
		gbc.fill = GridBagConstraints.HORIZONTAL;
		gb.setConstraints(txtBaseFileEstimatorMeans, gbc); pnlControls.add(txtBaseFileEstimatorMeans);
		gbc.gridx = 5; gbc.gridy = 1;
		gbc.weightx = 0;
		gbc.fill = GridBagConstraints.NONE;
		gb.setConstraints(btnBaseFileEstimatorMeans, gbc); pnlControls.add(btnBaseFileEstimatorMeans);

		gbc.gridheight = 2;
		gbc.gridx = 6; gbc.gridy = 0;
		gb.setConstraints(pnlLogLin, gbc); pnlControls.add(pnlLogLin);

		gbc.weightx = 0;
		gbc.gridheight = 1;
		gbc.gridx = 7; gbc.gridy = 0;
		gbc.fill = GridBagConstraints.HORIZONTAL;
		gb.setConstraints(btnRecalc, gbc); pnlControls.add(btnRecalc);

		gbc.weightx = 0;
		gbc.gridheight = 1;
		gbc.gridx = 7; gbc.gridy = 1;
		gbc.fill = GridBagConstraints.HORIZONTAL;
		gb.setConstraints(btnShow, gbc); pnlControls.add(btnShow);
		
//		this.showLegends();

	}


	public void showLegends()
	{
		glassPane.setVisible(true);
		glassPane.setLayout(null);
		glassPane.removeAll();
		glassPane.add(legend.getComponent());
		glassPane.add(legendRMS.getComponent());
		glassPane.add(legendTime.getComponent());

		legend.setLocation(100, 100); legend.setSize(100, 100);
		legendRMS.setLocation(200, 100); legendRMS.setSize(100, 100);
		legendTime.setLocation(200, 100); legendTime.setSize(100, 100);

		if (mouseInLegend != null)
			legend.getComponent().removeMouseMotionListener(mouseInLegend);
		mouseInLegend = new MouseInputAdapter() {
			public void mouseDragged(MouseEvent e) {
				Component compSource = (Component) e.getSource();
				e.translatePoint(compSource.getLocation().x, compSource.getLocation().y);
				compSource.setLocation(e.getPoint());
			}
		};
		legend.getComponent().addMouseMotionListener(mouseInLegend);

		if (mouseInLegendRMS != null)
			legendRMS.getComponent().removeMouseMotionListener(mouseInLegendRMS);
		mouseInLegendRMS = new MouseInputAdapter() {
			public void mouseDragged(MouseEvent e) {
				Component compSource = (Component) e.getSource();
				e.translatePoint(compSource.getLocation().x, compSource.getLocation().y);
				compSource.setLocation(e.getPoint());
			}
		};
		legendRMS.getComponent().addMouseMotionListener(mouseInLegendRMS);

		if (mouseInLegendTime != null)
		legendTime.getComponent().removeMouseMotionListener(mouseInLegendTime);
		mouseInLegendTime = new MouseInputAdapter() {
			public void mouseDragged(MouseEvent e) {
				Component compSource = (Component) e.getSource();
				e.translatePoint(compSource.getLocation().x, compSource.getLocation().y);
				compSource.setLocation(e.getPoint());
			}
		};
		legendTime.getComponent().addMouseMotionListener(mouseInLegendTime);

		MouseListener [] arrayListMouse = glassPane.getMouseListeners();
		for (int i = 0; i < arrayListMouse.length; i++)
			glassPane.removeMouseListener(arrayListMouse[i]);
		MouseMotionListener [] arrayListMotion = glassPane.getMouseMotionListeners();
		for (int i = 0; i < arrayListMotion.length; i++)
			glassPane.removeMouseMotionListener(arrayListMotion[i]);

		RedispatchMouseEvents mouseRedisp = new RedispatchMouseEvents(contentPane, glassPane);
		glassPane.addMouseListener(mouseRedisp);
		glassPane.addMouseMotionListener(mouseRedisp);
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


	public void estimationPerformed(EstimationStatisticsEvent e)
	{
		if (e.getSourceStatsComponent() != this)
			bHasValuesCalculated = false;
	}


	public void setEstimatesEngine(EstimatesEngine estEng)
	{
		this.estEngine = estEng;
	}


	public BlockUI getUI()
	{
		return this;
	}


	class RunWorkInterface implements Runnable 
	{
		private Thread threadWork = null;
		private boolean bWorkDone = false;
		private boolean bWorkStart = false;

		public void workStart()
		{
			bWorkStart = true;
		}

		public void workDone()
		{
			bWorkDone = true;
		}

		public void setWorkThread(Thread threadWork)
		{
			this.threadWork = threadWork;
		}
		
		public void run()
		{
			// GUI durante a execução.
			final JDialog diag = new JDialog();
			JLabel lblInfo = new JLabel("Procurando solução ...");
			final JProgressBar prgBar = new JProgressBar(JProgressBar.HORIZONTAL);
			JPanel buttonsHolder = new JPanel();
			JButton resumeButton = new JButton("Continuar");
			JButton pauseButton = new JButton("Pausar");
			JButton stopButton = new JButton("Abortar");

			prgBar.setMinimum(0);
			prgBar.setMaximum(20);

			diag.getContentPane().setLayout(new BorderLayout());
			buttonsHolder.setLayout(new GridLayout(1, 3));
			diag.getContentPane().add(lblInfo, BorderLayout.NORTH);
			diag.getContentPane().add(prgBar, BorderLayout.CENTER);
			buttonsHolder.add(resumeButton);
			buttonsHolder.add(pauseButton);
			buttonsHolder.add(stopButton);
			diag.getContentPane().add(buttonsHolder, BorderLayout.SOUTH);
			diag.pack();
			GUIUtil.getDefaultInstance().centerWindow(diag);
			
			Runnable rnnbl = new Runnable() {
							public void run()
							{	
								try
								{
									while (!bWorkDone)
									{
										int prgVal = prgBar.getValue();
										if (prgVal < prgBar.getMaximum())
											prgBar.setValue(prgVal + 1);
										else
											prgBar.setValue(0);
					
										Thread.sleep(100);
									}
								}
								catch(Exception exp)
								{
									JErrorMessageBox.show(exp, "Estimação parada a força.");
									threadWork.stop();
								}
								diag.setVisible(false);
							}
						};


			while (!bWorkStart)
			{
				try
				{
					Thread.sleep(1000);
				}
				catch (Exception e)
				{
				}
			}
			Thread thrdInternalThread = new Thread(rnnbl);
			thrdInternalThread.start();
			
			diag.setModal(true);
			diag.show();
		}		
	}


	class RunWork implements Runnable 
	{
		private RunWorkInterface runWorkInterface = null;
		
		public void setWorkInterface(RunWorkInterface runWorkInterface)
		{
			this.runWorkInterface = runWorkInterface;
		}
		
		public void run()
		{	
			try
			{
				calculate(runWorkInterface);
				showGraph();
			}
			catch(Exception exp)
			{
				exp.printStackTrace();
				JErrorMessageBox.show(exp, "Estimação parada a força.");
			}
		}
	}


	private void doCalculation()
	{
		RunWork runWork = new RunWork();
		RunWorkInterface runWorkInterface = new RunWorkInterface();
			
		Thread thrdWork = new Thread(runWork);
		Thread thrdWorkInterface = new Thread(runWorkInterface);

		runWorkInterface.setWorkThread(thrdWork);
		runWork.setWorkInterface(runWorkInterface);
			
		thrdWork.start();
		thrdWorkInterface.start();
	}


	class CalcActionListener implements ActionListener
	{
		public void actionPerformed(ActionEvent e)
		{
			doCalculation();
		}
	}


	class ShowActionListener implements ActionListener
	{
		public void actionPerformed(ActionEvent e)
		{
			showGraph();
		}
	}


	class ToggleLogListener implements ActionListener
	{
		public void actionPerformed(ActionEvent e)
		{
			showGraph();
		}
	}


	public void estimationIterationPerformed(AbstractDoubleVector [] arrayRealPoses, AbstractDoubleVector [][] arrayMeans, AbstractDoubleSquareMatrix [][] arrayCovars, int itCount, int estCount, int stepCount)
	{
		int countComponents = arrayMeans[0][0].dimension();
		if (arrayRMSMeansPerComponent == null || 
			arrayRMSMeansPerComponent.length != estCount ||
			arrayRMSMeansPerComponent[0].length != stepCount ||
			arrayRMSMeansPerComponent[0][0].length != countComponents)
		{
			arrayRMSMeansPerComponent = new double[estCount][stepCount][countComponents];
			arrayRMSMeansTotal = new double[estCount][stepCount];
			arrayRMSMeansTotalMean = new double[estCount];
		}

		for (int idxEstimator = 0; idxEstimator < estCount; idxEstimator++)
		{
			for (int idxStep = 0; idxStep < stepCount; idxStep++)
			{
				for (int idxComponent = 0; idxComponent < countComponents; idxComponent++)
				{
					double valTmp = arrayMeans[idxEstimator][idxStep].getComponent(idxComponent);
					double valReal = arrayRealPoses[idxStep].getComponent(idxComponent);
					arrayRMSMeansPerComponent[idxEstimator][idxStep][idxComponent] += Math.sqrt((valTmp - valReal)*(valTmp - valReal));
				}
			}
		}

		if (holderBaseFileEstimatorResultsPerIt.get() != null)
		{
			try
			{
				List listEsts = estEngine.getEstimators();
				String strFileBase = (String) holderBaseFileEstimatorResultsPerIt.get();
				
				for (int idxEstimator = 0; idxEstimator < estCount; idxEstimator++)
				{
					FileOutputStream flOut = new FileOutputStream(new File((strFileBase + "_" + itCount + "_" + ((Estimator) listEsts.get(idxEstimator)).getInstanceName()).replace(':', '_')));
					BufferedOutputStream bOut = new BufferedOutputStream(flOut);
					for (int idxStep = 0; idxStep < stepCount; idxStep++)
					{
						StringBuffer strBuff = new StringBuffer();
						int idxComponent = 0; 
						for (; idxComponent < countComponents - 1; idxComponent++)
						{
							strBuff.append(arrayMeans[idxEstimator][idxStep].getComponent(idxComponent));
							strBuff.append(",");
						}
						for (; idxComponent < countComponents; idxComponent++)
						{
							strBuff.append(arrayMeans[idxEstimator][idxStep].getComponent(idxComponent));
							strBuff.append(";\n");
						}
						bOut.write(strBuff.toString().getBytes());
					}
					bOut.flush(); bOut.close();
					flOut.flush(); flOut.close(); 
				}
			}
			catch (IOException e)
			{
				log.error("erro", e);
			}
		}

		if (itCount == 0 && holderBaseFileEstimatorResultsPerIt.get() != null)
		{
			String strFileBase = (String) holderBaseFileEstimatorResultsPerIt.get();
			try
			{
				FileOutputStream flOut = new FileOutputStream(new File((strFileBase + "_real").replace(':', '_')));
				BufferedOutputStream bOut = new BufferedOutputStream(flOut);
				for (int idxStep = 0; idxStep < stepCount; idxStep++)
				{
					int idxComponent = 0; 
					StringBuffer strBuff = new StringBuffer();
					for (; idxComponent < countComponents - 1; idxComponent++)
					{
						strBuff.append(arrayRealPoses[idxStep].getComponent(idxComponent));
						strBuff.append(",");
					}
					for (; idxComponent < countComponents; idxComponent++)
					{
						strBuff.append(arrayRealPoses[idxStep].getComponent(idxComponent));
						strBuff.append("\n");
					}
					bOut.write(strBuff.toString().getBytes());
				}
				bOut.flush(); bOut.close();
				flOut.flush(); flOut.close(); 
			}
			catch (IOException e)
			{
				log.error("erro", e);
/*				
				JOptionPane.showMessageDialog(null, 
						"Arquivo de Iterações não existe: " + strFileBase,
						"Arquivo de Iterações não existe: " + strFileBase,
						JOptionPane.ERROR_MESSAGE);
//*/						
			}
		}
	}


	protected void calculate(RunWorkInterface runWorkInterface)
	{
		bHasValuesCalculated = false;

		String strCountIt = txtFldCountIt.getText();
		int itTemp = -1;
		try
		{
			itTemp = Integer.parseInt(strCountIt);
		}
		catch (Exception e)
		{
		}

		if (itTemp < 1)
		{
			JOptionPane.showMessageDialog(null, 
					"O número de iterações entrado é inválido",
					"O número de iterações entrado é inválido",
					JOptionPane.ERROR_MESSAGE);
		}
		else
		{
			itNumber = itTemp;
		}

		if(holderBaseFileEstimatorResultsPerIt.get() == null &&
			txtBaseFileEstimatorResultsPerIt.getText() != null &&
			!txtBaseFileEstimatorResultsPerIt.getText().equals(""))
		holderBaseFileEstimatorResultsPerIt.set(txtBaseFileEstimatorResultsPerIt.getText());

		if(holderBaseFileEstimatorMeans.get() == null &&
			txtBaseFileEstimatorMeans.getText() != null &&
			!txtBaseFileEstimatorMeans.getText().equals(""))
		holderBaseFileEstimatorMeans.set(txtBaseFileEstimatorMeans.getText());

		if(holderBaseFileEstimatorResultsPerIt.get() == null)
		{
			JOptionPane.showMessageDialog(null, 
					"Arquivo de Iterações não configurado",
					"Arquivo de Iterações não configurado",
					JOptionPane.ERROR_MESSAGE);
		}
		if(holderBaseFileEstimatorMeans.get() == null)
		{
			JOptionPane.showMessageDialog(null, 
					"Arquivo de Médias não configurado",
					"Arquivo de Médias não configurado",
					JOptionPane.ERROR_MESSAGE);
		}
		
		runWorkInterface.workStart();
		estEngine.processData(itNumber, new NoiseControlInfo(), this);

//		arrayMeans = estEngine.getEstimatesMean();
//		arrayCovars = estEngine.getEstimatesCovariance();
		arrayRealPoses = estEngine.getRealValues();
		arrayTimes = estEngine.getTotalTimes();

		int countComponents = arrayRMSMeansPerComponent[0][0].length;

		for (int idxEstimator = 0; idxEstimator < arrayRMSMeansTotal.length; idxEstimator++)
		{
			for (int idxStep = 0; idxStep < arrayRMSMeansTotal[0].length; idxStep++)
			{
				for (int idxComponent = 0; idxComponent < countComponents; idxComponent++)
				{
					arrayRMSMeansPerComponent[idxEstimator][idxStep][idxComponent] /= itNumber;
					arrayRMSMeansTotal[idxEstimator][idxStep] += arrayRMSMeansPerComponent[idxEstimator][idxStep][idxComponent];
					arrayRMSMeansTotalMean[idxEstimator] += arrayRMSMeansPerComponent[idxEstimator][idxStep][idxComponent];
				}
			}
			arrayRMSMeansTotalMean[idxEstimator] /= arrayRMSMeansTotal[0].length;
		}
		
		if(holderBaseFileEstimatorMeans.get() != null)
		{
			String strFileBase = (String) holderBaseFileEstimatorMeans.get();
			try
			{
				List listEsts = estEngine.getEstimators();
				for (int idxEstimator = 0; idxEstimator < arrayRMSMeansTotal.length; idxEstimator++)
				{
					FileOutputStream flOut = new FileOutputStream(new File((strFileBase + "_" + ((Estimator) listEsts.get(idxEstimator)).getInstanceName()).replace(':', '_')));
					BufferedOutputStream bOut = new BufferedOutputStream(flOut);
					for (int idxStep = 0; idxStep < arrayRMSMeansTotal[0].length; idxStep++)
					{
						StringBuffer strBuff = new StringBuffer();
						int idxComponent = 0; 
						for (; idxComponent < countComponents - 1; idxComponent++)
						{
							strBuff.append(arrayRMSMeansPerComponent[idxEstimator][idxStep][idxComponent]);
							strBuff.append(",");
						}
						for (; idxComponent < countComponents; idxComponent++)
						{
							strBuff.append(arrayRMSMeansPerComponent[idxEstimator][idxStep][idxComponent]);
							strBuff.append("\n");
						}
						bOut.write(strBuff.toString().getBytes());
					}
					bOut.flush(); bOut.close();
					flOut.flush(); flOut.close(); 
				}
			}
			catch (IOException e)
			{
				log.error("erro", e);
				JOptionPane.showMessageDialog(null, 
						"Arquivo de Médias não existe: " + strFileBase,
						"Arquivo de Médias não existe: " + strFileBase,
						JOptionPane.ERROR_MESSAGE);
			}
		}
		runWorkInterface.workDone();

		bHasValuesCalculated = true;
//		showLegends();
	}


	protected void showGraph()
	{
		if (!bHasValuesCalculated)
		{
			JOptionPane.showMessageDialog(null, 
					"Não há valores disponíveis. Calcule primeiro.",
					"Não há valores disponíveis. Calcule primeiro.",
					JOptionPane.ERROR_MESSAGE);
			return;
		}

		HashMap mapEstRMSs = new HashMap();
		HashMap mapEstInfos = new HashMap();
		HashMap mapEstTimes = new HashMap();
		List listInfos = estEngine.getEstimatorRendererInfos();
		List listEsts = estEngine.getEstimators();
		for (int i = 0; i < listEsts.size(); i++)
		{
			mapEstInfos.put(listInfos.get(i), ((Estimator) listEsts.get(i)).getInstanceName());
			mapEstRMSs.put(listInfos.get(i), arrayRMSMeansTotalMean[i] + "");
			mapEstTimes.put(listInfos.get(i), arrayTimes[i] + "");
		}

		legend.setRendererInfos(mapEstInfos);
		legendRMS.setRendererInfos(mapEstRMSs);
		legendTime.setRendererInfos(mapEstTimes);
		
		legend.resetUI();
		legendRMS.resetUI();
		legendTime.resetUI();

		chart = null;

		XYSeriesCollection seriesColl = new XYSeriesCollection();
		for (int idxEst = 0; idxEst < listEsts.size(); idxEst++)
		{
			EstimatorRendererInfo estInfo = (EstimatorRendererInfo) listInfos.get(idxEst);
			
			XYSeries series = new XYSeries(((Estimator) listEsts.get(idxEst)).getInstanceName());
			for (int idxSteps = 0; idxSteps < arrayRMSMeansTotal[idxEst].length; idxSteps++) 
			{
				series.add(idxSteps, arrayRMSMeansTotal[idxEst][idxSteps]);
			}
			seriesColl.addSeries(series);
		}

//		chart = ChartFactory.createScatterPlot("RMS", "passo", "RMS", seriesColl, PlotOrientation.VERTICAL, true, true, false);
		chart = ChartFactory.createXYLineChart("RMS", "passo", "RMS", seriesColl, PlotOrientation.VERTICAL, true, true, false);
		chart.setBackgroundPaint(Color.yellow);
		chart.setBorderPaint(Color.blue);
		XYPlot plot = chart.getXYPlot();
		plot.setRenderer(new StandardXYItemRenderer(StandardXYItemRenderer.LINES));
		plot.setDomainCrosshairVisible(false);
		plot.setRangeCrosshairVisible(false);
//		plot.setDomainCrosshairPaint(Color.red);
		plot.setDomainGridlinesVisible(true);
		plot.setWeight(50);
//			XYDotRenderer dotRndr = new XYDotRenderer();
//			XYAreaRenderer areaRndr = new XYAreaRenderer(XYAreaRenderer.SHAPES);
//			XYAreaRenderer areaRndr = new XYAreaRenderer(XYAreaRenderer.AREA_AND_SHAPES);
//			XYAreaRenderer areaRndr = new XYAreaRenderer(XYAreaRenderer.AREA);
//			XYAreaRenderer areaRndr = new XYAreaRenderer(XYAreaRenderer.SHAPES_AND_LINES);
//		XYStepAreaRenderer areaRndr = new XYStepAreaRenderer(XYStepAreaRenderer.SHAPES);
//			areaRndr.setShape(new Rectangle(10, 10));
//			areaRndr.setBasePaint(Color.red);
//		areaRndr.setPaint(Color.blue);
//			areaRndr.setOutline(false);
//		areaRndr.setShapesFilled(true);
//		plot.setRenderer(areaRndr);

		NumberAxis domainAxis = (NumberAxis) plot.getDomainAxis();
		domainAxis.setAutoRangeIncludesZero(true);

//		if ("log".equals(optLog.getActionCommand()))
		if (optLog.isSelected())
		{
			LogarithmicAxis imgAxis = new LogarithmicAxis("RMS");
			imgAxis.setAllowNegativesFlag(true);
			plot.setRangeAxis(imgAxis);
		}
		else
		{
			NumberAxis imgAxis = (NumberAxis) plot.getRangeAxis();
			imgAxis.setAutoRangeIncludesZero(true);
		}
		
//*
		if (chartPanel == null)
		{
			Dimension dimThis = contentPane.getSize();
			chartPanel = new ChartPanel(chart);
//			chartPanel.setPreferredSize(new java.awt.Dimension(dimThis.width, dimThis.height));
//			chartPanel.setPreferredSize(new java.awt.Dimension(500, 500));
			chartPanel.setMouseZoomable(true, false);
			chartPanel.setVerticalAxisTrace(false);
			chartPanel.setHorizontalAxisTrace(false);
			chartPanel.setVerticalZoom(true);
			chartPanel.setHorizontalZoom(true);
			pnlChartBase.setLayout(new BorderLayout());
			pnlChartBase.add(new JScrollPane(chartPanel), BorderLayout.CENTER);
		}
		else
			chartPanel.setChart(chart);
//*/			
	}


	class RedispatchMouseEvents implements MouseListener, MouseMotionListener
	{
		private Component glassPane = null;
		private Component contentPane = null;
		 
		public RedispatchMouseEvents(Component contentPane, Component glassPane)
		{
			this.contentPane = contentPane;
			this.glassPane = glassPane;
		}

		// catch all mouse events and redispatch them
		public void mouseMoved(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseDragged(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseClicked(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseEntered(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseExited(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mousePressed(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}
		public void mouseReleased(MouseEvent e) {
			redispatchMouseEvent(e, false);
		}


		private void redispatchMouseEvent(MouseEvent e, boolean repaint)
		{
			// get the mouse click point relative to the content pane
			Point containerPoint = SwingUtilities.convertPoint(glassPane, e.getPoint(), contentPane);

			// find the component that under this point
			Component component = SwingUtilities.getDeepestComponentAt(
													contentPane,
													containerPoint.x,
													containerPoint.y);

				// return if nothing was found
				if (component == null)
					return;

				// convert point relative to the target component
				Point componentPoint = SwingUtilities.convertPoint(glassPane, e.getPoint(), component);

				// redispatch the event
				component.dispatchEvent(new MouseEvent(component,
												e.getID(),
												e.getWhen(),
												e.getModifiers(),
												componentPoint.x,
												componentPoint.y,
												e.getClickCount(),
												e.isPopupTrigger(),
												e.getButton()));
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
//				holderFile.set((new File(strFileName)).getCanonicalPath());
				holderFile.set(strFileName);
			}
		}
	}
}
