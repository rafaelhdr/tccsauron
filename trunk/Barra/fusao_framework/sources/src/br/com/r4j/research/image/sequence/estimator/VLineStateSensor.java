package br.com.r4j.research.image.sequence.estimator;

import java.awt.image.BufferedImage;
import java.io.File;
import java.util.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.research.image.sequence.featurematch.vline.ExtractionPhaseMatcher;
import br.com.r4j.research.image.sequence.featurematch.vline.LineExtrationResult;
import br.com.r4j.research.vline.*;
import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RobotPlayerEvent;
import br.com.r4j.robosim.estimator.BaseModel;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.Sensor;


public class VLineStateSensor implements Sensor, EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(VLineStateSensor.class.getName());


	private boolean bHasData = false;
	public int imgWidth = -1;
	public int imgHeight = -1;
	private VLineMapSensor snsMap = null;
	private CameraModel camModel = null;
	private boolean bNewReadings = false;
	private boolean bNewReadings2 = false;


	public VLineStateSensor()
	{
		super();
	}


	public String getName()
	{
		return "VLines Sate Sensor";
	}


	public CameraModel getCameraModel()	{return camModel;}
	public void setCameraModel(CameraModel camModel)
	{
		this.camModel = camModel;
	}

	public void setMapSensor(VLineMapSensor sns)
	{
		this.snsMap = sns;
	}

	
	private int itCount = 0; // de-de-debug!
	public void setReadings(int [] data, int w, int h)
	{
		log.debug("Sonar Sate Sensor: " + itCount);
		if (data != null)
		{
			bNewReadings = true;
			bNewReadings2 = true;
			if (!snsMap.isBeingUsed())
			{
				log.debug("calling other sensor: " + itCount);
				snsMap.setReadings(data, w, h);
				snsMap.setBeingUsed(false);
			}
			bHasData = true;
			itCount++;
		}
		else
			bHasData = false;
	}


	public void dataAvailable()
	{
		// não precisa fazer nada ...
		if (bNewReadings)
		{
			bNewReadings = false;
		}
		else
		{
			bNewReadings2 = false;
		}
	}


	public int getDataDimension(BaseModel baseModel)
	{
		VLineMap lineMap = ((VLinesStateSensorModel) baseModel).getLineMap();
//		return newLineProjs.arrayProjMeasures.length;
		return lineMap.getNumberOfStateLines();
//		return snsMap.getDataDimension(baseModel);
	}


	public AbstractDoubleSquareMatrix getDataCovariance(BaseModel snsModel)
	{
		AbstractDoubleSquareMatrix covar = new DoubleSquareMatrix(this.getDataDimension(snsModel));
		return covar;
	}


	public void getData(AbstractDoubleVector output, BaseModel baseModel)
	{
		VLineMap lineMap = ((VLinesStateSensorModel) baseModel).getLineMap();
		log.debug("sensor:getData: " + lineMap.getNumberOfStateLines() + ", " + lineMap.getNumberOfMappedLines());
		Iterator itLines = lineMap.getStateLineIndexIterator();
		int countLineIdx = 0;
		while (itLines.hasNext())
		{
			VLine line = lineMap.nextLine(itLines);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);

			output.setComponent(countLineIdx, lineProj.getU());
			countLineIdx++;
		}
	}


/*
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
//*/

	public boolean hasNewData(BaseModel baseModel, Estimator est)
	{
		log.debug("VLineStateSensor->hasNewData: " + bHasData);
		log.debug("((VLinesStateSensorModel) baseModel).getDataDimension(): " + ((VLinesStateSensorModel) baseModel).getDataDimension());
		if  (bHasData && bNewReadings2)
		{
			((VLinesStateSensorModel) baseModel).doDataAssociation(est.getMean(), est.getCovariance());
			return ((VLinesStateSensorModel) baseModel).getDataDimension() > 0;
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
		bHasData = false;
	}


	public void newPose(Pose2D pose)
	{
		// TODO Auto-generated method stub
		
	}
}


