package br.com.r4j.robosim;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.Iterator;

import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


public class RobotTrack implements EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(RobotTrack.class.getName());

	private ArrayList listPoses = null;
	private ArrayList listShapes = null;
	
	private String strNameMyName = null;

	private Shape shpSmall = null;

	private EstimatorRendererInfo info = null;
	
	private int currentStep = 0;
	

	public RobotTrack()
	{
		listPoses = new ArrayList();
		listShapes = new ArrayList();
		currentStep = -1;
		strNameMyName = "Pose Tracker";
	}


	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
		this.info = info;
		shpSmall = ShapesUtil.scaleShape(info.getShape(), 0.35);
	}

	
	private boolean bRendering = false;
	public void newPose(Pose2D pose)
	{
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 4) bRendering = false;
			else try {Thread.sleep(60);} catch (Exception e) {}
		}
		listPoses.add(pose);
		listShapes.add(ShapesUtil.rotateShape(shpSmall, pose.getTheta() - Math.PI/2));
		currentStep++;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		try
		{
			bRendering = true;
			Graphics2D g2d = e.getGraphics();
			g2d.setPaintMode();
			Iterator itPoses = listPoses.iterator();
			Iterator itShapes = listShapes.iterator();
			int i = 0;
			while (itShapes.hasNext())
			{
				if (currentStep < i)
					break;
				Shape shp = (Shape) itShapes.next();
				Pose2D pose = (Pose2D) itPoses.next();
				shp = e.translateAndMaintainShapeSize(pose.getX(), pose.getY(), shp);
				g2d.setColor(info.getColorFill());
				g2d.fill(shp);
				i++;
			}
			bRendering = false;
		}
		catch (RuntimeException e1)
		{
			log.error("erro inesperado: " + e1.getMessage(), e1);
		}
	}


	public void render(RendererEvent e)
	{
		try
		{
			bRendering = true;
			Iterator itPoses = listPoses.iterator();
			Iterator itShapes = listShapes.iterator();
			int i = 0;
			if (listPoses.size() > 0)
			{
				Graphics2D g2d = e.getGraphics();
				g2d.setPaintMode();
				Shape shp = (Shape) listShapes.get(currentStep);
				Pose2D pose = (Pose2D) listPoses.get(currentStep);
				shp = e.translateAndMaintainShapeSize(pose.getX(), pose.getY(), shp);
				g2d.setColor(info.getColorFill());
				g2d.fill(shp);
				i++;
			}
			bRendering = false;
		}
		catch (RuntimeException e1)
		{
			log.error("erro inesperado: " + e1.getMessage(), e1);
		}
	}


	public void erase(RendererEvent e)
	{
	}


	public String getName()
	{
		return strNameMyName;
	}


	public void setName(String a)
	{
		strNameMyName = a;
	}


	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
		
	}
}
