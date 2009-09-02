package br.com.r4j.research.image.sequence.estimator;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.*;
import java.awt.Graphics2D;
import javax.swing.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.research.image.sequence.featurematch.vline.ExtractionPhaseMatcher;
import br.com.r4j.research.image.sequence.featurematch.vline.LineExtrationResult;
import br.com.r4j.research.vline.*;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;

import br.com.r4j.gui.*;
import br.com.r4j.gui.swing.*;
import br.com.r4j.commons.util.*;
import br.com.r4j.gui.renderer.*;


public class VLineMapSensor implements Sensor, EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(VLineMapSensor.class.getName());
	private static Log logStats = LogFactory.getLog("estatisticas");
	private static Log logVisionModelCount = LogFactory.getLog("modelo_visao");
	private static Log logVisionExtractCount = LogFactory.getLog("extracao_visao");


	public int [] imgData = null;
	public int imgWidth = -1;
	public int imgHeight = -1;
	private VLineMap lineMap = null;
	public ExtractionPhaseMatcher extractionPhase = null;
	public LineExtrationResult newLineProjs = null;
	private boolean bBeingUsed = false;
	private CameraModel camModel = null;
	private boolean bNewReadings = false;

	private JDialog diagImage = null;


	private void createDialog(int imgWidth, int imgHeight)
	{
		diagImage = new JEscDialog(GUIFactory.getFrameForComponent(null), true);
		diagImage.setSize(imgWidth + 10, imgHeight + 50);
		diagImage.setModal(false);
		diagImage.pack();
		GUIFactory.centerDialog(diagImage);
		diagImage.show();
		diagImage.setSize(imgWidth + 10, imgHeight + 50);
	}


	private void drawImage(int [] imgData, int imgWidth, int imgHeight)
	{
		Graphics2D g2d = (Graphics2D) diagImage.getContentPane().getGraphics();
		if (g2d != null)
		{
			g2d.drawImage(ImageUtil.createBufferedImage(imgData, imgWidth, imgHeight, BufferedImage.TYPE_INT_RGB), 0, 0, null);
		}
//		diagImage.getContentPane().revalidate();
	}


	public VLineMapSensor()
	{
		super();
		extractionPhase = new ExtractionPhaseMatcher();
	}


	public String getName()
	{
		return "VLines Map Sensor";
	}


	public CameraModel getCameraModel()	{return camModel;}
	public void setCameraModel(CameraModel camModel)
	{
		this.camModel = camModel;
	}

	public void setVLineMap(VLineMap lineMap)
	{
		this.lineMap = lineMap;
	}

	
	public boolean isBeingUsed()
	{
		return bBeingUsed;
	}

	public void setBeingUsed(boolean b)
	{
		this.bBeingUsed = b;
	}

	
	private int itCount = 0; // de-de-debug!
	public void setReadings(int [] data, int w, int h)
	{
		if (log.isDebugEnabled())
			log.debug("Sonar Map Sensor: " + itCount);
		bBeingUsed = true;
		if (data != null)
		{
			bNewReadings = true;
			this.imgData = data;
			this.imgWidth = w;
			this.imgHeight = h;

			long tmTm = System.currentTimeMillis();
			extractionPhase.setItCount(itCount);
			newLineProjs = extractionPhase.lineExtraction(imgData, imgWidth, imgHeight);

			if (log.isDebugEnabled())
				log.debug("extraction: " + (System.currentTimeMillis() - tmTm));

			if (logStats.isDebugEnabled())
			{
				logStats.debug("VISÃO - EXTRAÇÃO:");
				logStats.debug("\ttempo: " + (System.currentTimeMillis() - tmTm));
				logStats.debug("\tcount: " + newLineProjs.arrayProjMeasures.length);
			}
			if (logVisionExtractCount.isDebugEnabled())
			{
				logVisionExtractCount.debug(newLineProjs.arrayProjMeasures.length + ";");
			}

			/*PRINTIMGS
			if (diagImage == null)
				this.createDialog(imgWidth, imgHeight);
			this.drawImage(imgData, imgWidth, imgHeight);
			//*/

			itCount++;
		}
	}


	public LineExtrationResult getLastResult()
	{
		return newLineProjs;
	}


	public void dataAvailable()
	{
		if (bNewReadings)
		{
			bNewReadings = false;
		}
		else
		{
			imgData = null;
		}
	}


	public int getDataDimension(BaseModel baseModel)
	{
		VLineMap lineMap = ((VLinesMapSensorModel) baseModel).getLineMap();
//		return newLineProjs.arrayProjMeasures.length;
		return lineMap.getNumberOfMappedLines();
	}


	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel snsModel)
	{
		AbstractDoubleSquareMatrix covar = new DoubleSquareMatrix(this.getDataDimension(snsModel));
		return covar;
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		VLineMap lineMap = ((VLinesMapSensorModel) baseModel).getLineMap();
//		log.debug("sensor:getData: " + lineMap.getNumberOfStateLines() + ", " + lineMap.getNumberOfMappedLines());
		int countLineIdx = 0;
		Iterator itLines = lineMap.getMappedLineIndexIterator();
		while (itLines.hasNext())
		{
			VLine line = lineMap.nextLine(itLines);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);

			if (log.isDebugEnabled())
				log.debug("lineProj.getU()): " + lineProj.getU());
			output.setComponent(countLineIdx, lineProj.getU());
			countLineIdx++;
		}
	}


	public int [] getImage()
	{
		return imgData;
	}
	
		
	public int getImageWidth()
	{
		return imgWidth;
	}


	public int getImageHeight()
	{
		return imgHeight;
	}


	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		if (log.isDebugEnabled())
			log.debug("imgData = " + imgData);
		if  (imgData != null)
		{
			((VLinesMapSensorModel) baseModel).doDataAssociation(est.getMean(), est.getCovariance());

			if (logStats.isDebugEnabled())
			{
				logStats.debug("VISÃO - MODELO:");
				logStats.debug("\tcount: " + ((VLinesMapSensorModel) baseModel).getDataDimension());
			}

			if (logVisionModelCount.isDebugEnabled())
				logVisionModelCount.debug(((VLinesMapSensorModel) baseModel).getDataDimension() + ";");

			return ((VLinesMapSensorModel) baseModel).getDataDimension() > 1;
		}
		else
		{
			return false;
		}
	}


	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
	}


	private int currentStep = -1;
	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
	}


	public void render(RendererEvent e)
	{
	}


	public void erase(RendererEvent e)
	{
	}


	public void actionCompleted(RobotPlayerEvent e)
	{
	}


	public void endOfActions(RobotPlayerEvent e)
	{
	}


	public void beginOfActions(RobotPlayerEvent e)
	{
	}


	public void actionsUpdated(RobotPlayerEvent e)
	{
	}


	public void reset()
	{
		imgData = null;
	}


	public void newPose(Pose2D pose)
	{
		// TODO Auto-generated method stub
		
	}
}


