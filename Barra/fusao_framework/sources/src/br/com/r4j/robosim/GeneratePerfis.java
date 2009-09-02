package br.com.r4j.robosim;

import java.awt.Color;
import java.awt.Component;
import java.awt.Frame;
import java.awt.Point;
import java.awt.Shape;
import java.awt.event.ActionEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.*;
import java.util.*;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.awt.image.BufferedImage;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JComponent;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputAdapter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;
import br.com.r4j.configurator.WebStartConfigurator;
import br.com.r4j.gui.BlockUI;
import br.com.r4j.gui.CSSLikeGUIStyleManager;
import br.com.r4j.gui.GUIStyleManager;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.gui.SplashScreen;
import br.com.r4j.robosim.estimator.EstimatesEngine;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.robosim.estimator.SensorModel;
import br.com.r4j.robosim.estimator.impl.BaseEstimator;
import br.com.r4j.robosim.estimator.provider.EstimationObjectsProvider;
import br.com.r4j.robosim.gui.ActionPlayerControlUI;
import br.com.r4j.robosim.gui.ApplicationUI;
import br.com.r4j.robosim.gui.Legend;
import br.com.r4j.robosim.gui.OnlineRMSError;
import br.com.r4j.robosim.gui.OnlineUI;
import br.com.r4j.robosim.gui.RMSStatsUI;
import br.com.r4j.robosim.gui.RandomNoiseRMSStatsUI;
import br.com.r4j.robosim.gui.SimulatorUI;
import br.com.r4j.robosim.gui.StatsUI;
import br.com.r4j.robosim.gui.WorldUI;

import br.com.r4j.image.operation.threebandpacked.spatialfilter.*;
import br.com.r4j.research.vline.*;
import br.com.r4j.commons.util.Arrays;


public class GeneratePerfis
{
	private static Log log = LogFactory.getLog(GeneratePerfis.class.getName());
	private static Log logOut = LogFactory.getLog("out");

	public static void main(String [] args)
	{
//		new br.com.r4j.image.operation.imageop.GradientOp();
		try
		{
			WebStartConfigurator.createConfigurator("conf/genperf-conf.xml");

			Configurator conf = Configurator.getInstance();
			PropertiesHolder props = conf.getPropertiesHolder();


			GeneratePerfis app = new GeneratePerfis();
			app.configure(props);
			app.generate();
		}
		catch (ConfiguratorException e)
		{
			e.printStackTrace();
			return;
		}
		catch (Exception e)
		{
			e.printStackTrace();
			return;
		}
	}


	public GeneratePerfis()
	{
	}


	private File flComPerfis = null;
	public void configure(PropertiesHolder props) throws ConfiguratorException
	{
		flComPerfis = props.getFileProperty("gen-perfil/file");
	}


	private int [] arrayTmp = null;
	public void generate()
	{
		BufferedImage buffImg = ImageUtil.getImageBMP(flComPerfis);
		int [] imgData = ImageUtil.getThreeBandPackedData(buffImg);
		int imgWidth = buffImg.getWidth();
		int imgHeight = buffImg.getHeight();
		arrayTmp = new int[imgData.length];

		SobelVerticalLinesFilter vetFilter = new SobelVerticalLinesFilter();
		vetFilter.setItCount(9999);
		vetFilter.operate(imgData, arrayTmp, imgWidth, imgHeight);

		TreeSet setMeasuresTmp = new TreeSet();
		List listLines = vetFilter.getLines();
		for (int i = 0; i < listLines.size(); i++)
		{
			LineSegmentCollection lCol = (LineSegmentCollection) listLines.get(i);
			VLineProj vLineP = new VLineProj(lCol, 1);
			vLineP.calculatePerfil(imgData, imgWidth, imgHeight);
			VLinePerfil perfil = vLineP.getPerfil();

			logOut.debug("----- projeção: " + vLineP.getU());
			logOut.debug("mean R: " + perfil.getMeanR1() + ", " + perfil.getMeanR2() + ", " + (perfil.getMeanR1() - perfil.getMeanR2()));
			logOut.debug("mean G: " + perfil.getMeanG1() + ", " + perfil.getMeanG2() + ", " + (perfil.getMeanG1() - perfil.getMeanG2()));
			logOut.debug("mean B: " + perfil.getMeanB1() + ", " + perfil.getMeanB2() + ", " + (perfil.getMeanB1() - perfil.getMeanB2()));
			logOut.debug("perfil R: " + Arrays.toString(perfil.getPerfilRed(), perfil.getPerfilBlue().length, 3));
			logOut.debug("perfil G: " + Arrays.toString(perfil.getPerfilGreen(), perfil.getPerfilBlue().length, 3));
			logOut.debug("perfil B: " + Arrays.toString(perfil.getPerfilBlue(), perfil.getPerfilBlue().length, 3));
		}
	}
}
