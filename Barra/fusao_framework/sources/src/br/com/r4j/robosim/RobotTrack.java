package br.com.r4j.robosim;

import java.awt.Graphics2D;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.commons.util.TraceUtil;

import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;


public class RobotTrack implements EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(RobotTrack.class.getName());

	private ArrayList listPoses = null;
	private ArrayList listShapes = null;
	
	private String strNameMyName = null;

	private Shape shpSmall = null;

	private EstimatorRendererInfo info = null;
	
	private int currentStep = 0;

	private int jump = 0;

	private int jumpOffset = 0;
	private static int jumpOffsetGen = 0;


	public RobotTrack()
	{
		listPoses = new ArrayList();
		listShapes = new ArrayList();
		currentStep = -1;
		strNameMyName = "Pose Tracker";
		jumpOffset = jumpOffsetGen;
		jumpOffsetGen += 8;
	}


	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
		this.info = info;
		shpSmall = ShapesUtil.scaleShape(info.getShape(), 0.54);

		PropertiesHolder props = Configurator.getInstance().getPropertiesHolder();
		if (props.containsProperty("/robosim/tracker/jump"))
			jump = props.getIntegerProperty("/robosim/tracker/jump").intValue();
	
	}

	
	private boolean bRendering = false;
	public void newPose(Pose2D pose)
	{
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 12) bRendering = false;
			else try {Thread.sleep(60);} catch (Exception e) {}
		}
		listPoses.add(pose);
//		listShapes.add(ShapesUtil.rotateShape(shpSmall, pose.getTheta() - Math.PI/2));
		listShapes.add(ShapesUtil.rotateShape(shpSmall, pose.getTheta()));
		currentStep++;
		log.debug(this.hashCode() + "," + strNameMyName + ": TRACKER("+currentStep+"): " + pose);
//		log.debug(TraceUtil.currentStackTrace("\n"));
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
			Pose2D poseLast = null;
			while (itShapes.hasNext())
			{
				if (currentStep < i)
					break;

				Shape shp = (Shape) itShapes.next();
				Pose2D pose = (Pose2D) itPoses.next();

				if (poseLast != null)
				{
					g2d.setColor(info.getColorFill());
					g2d.drawLine((int) poseLast.getX(), (int) poseLast.getY(), (int) pose.getX(), (int) pose.getY());
				}
				if ((i+jumpOffset)%(jump+1) == 0)
				{
					shp = e.translateAndMaintainShapeSize(pose.getX(), pose.getY(), shp);
					g2d.setColor(info.getColorFill());
					g2d.fill(shp);
				}
				poseLast = pose;
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
			if (listPoses.size() > 1)
			{
				Pose2D pose = (Pose2D) listPoses.get(currentStep);
				Pose2D poseLast = (Pose2D) listPoses.get(currentStep-1);
				if (poseLast != null)
				{
					Graphics2D g2d = e.getGraphics();
					g2d.setPaintMode();
					g2d.setColor(info.getColorFill());
					g2d.setColor(info.getColorFill());
					g2d.drawLine((int) poseLast.getX(), (int) poseLast.getY(), (int) pose.getX(), (int) pose.getY());
				}
			}
			if (listPoses.size() > 0)
			{
				if ((currentStep+jumpOffset)%(jump+1) == 0)
				{
					Graphics2D g2d = e.getGraphics();
					g2d.setPaintMode();
					Shape shp = (Shape) listShapes.get(currentStep);
					Pose2D pose = (Pose2D) listPoses.get(currentStep);
					shp = e.translateAndMaintainShapeSize(pose.getX(), pose.getY(), shp);
					g2d.setColor(info.getColorFill());
					g2d.fill(shp);
				}
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
		log.debug(this.hashCode() + "," + strNameMyName + ": TRACKER(ste step): " + currentStep);
		
	}
}
