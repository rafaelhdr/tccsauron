package br.com.r4j.robosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;


public class RobotPose implements EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(RobotPose.class.getName());

	private ArrayList listPoses = null;
	private ArrayList listShapes = null;

	private Pose2D pose = null;
	private Pose2D poseLast = null;
	
	private Shape shapeCurrent = null;
	private Shape shapeLast = null;

	private String strNameMyName = null;

	private EstimatorRendererInfo info = null;

	private int currentStep = 0;


	public RobotPose()
	{
		listPoses = new ArrayList();
		listShapes = new ArrayList();
		currentStep = -1;

		strNameMyName = "Pose Tracker";
	}


	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
		this.info = info;
	}


	private boolean bRendering = false;
	public void newPose(Pose2D pose)
	{
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 4) bRendering = false;
			else try {Thread.sleep(20);} catch (Exception e) {}
		}
		currentStep++;
//		log.debug("newPose("+strNameMyName+"): " + pose);
		this.pose = pose;
//		shapeCurrent = ShapesUtil.createTriangle(20, 12, 20, true, pose.getX(), (int) pose.getY(), (int) pose.getTheta());
		shapeCurrent = ShapesUtil.rotateShape(info.getShape(), pose.getTheta());
//		shapeCurrent = info.getShape();
		listPoses.add(pose);
		listShapes.add(shapeCurrent);
//		log.debug("shapeCurrent: " + shapeCurrent);
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;
		if (shapeCurrent != null)
		{
			Graphics2D g2d = e.getGraphics();
			g2d.setPaintMode();
	
			Shape shpShp = e.translateAndMaintainShapeSize(pose.getX(), pose.getY(), shapeCurrent);
			BasicStroke strokeObjectOutline = new BasicStroke(2.5f);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(info.getColorFill());
			g2d.fill(shpShp);
			g2d.setColor(info.getColorBorder());
			g2d.draw(shpShp);

			shapeLast = shapeCurrent;
			poseLast = pose; 
		}
		bRendering = false;
	}


	public void render(RendererEvent e)
	{
		bRendering = true;
		if (shapeCurrent != null)
		{
			Graphics2D g2d = e.getGraphics();
	
			Shape shpShp = e.translateAndMaintainShapeSize(pose.getX(), pose.getY(), shapeCurrent);
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setXORMode(Color.white);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(info.getColorFill());
			g2d.fill(shpShp);
			g2d.setColor(info.getColorBorder());
			g2d.draw(shpShp);

			shapeLast = shapeCurrent;
			poseLast = pose; 
		}
		bRendering = false;
	}


	public void erase(RendererEvent e)
	{
		bRendering = true;
		if (shapeLast != null)
		{
			Graphics2D g2d = e.getGraphics();
	
			Shape shpShp = e.translateAndMaintainShapeSize(poseLast.getX(), poseLast.getY(), shapeLast);
			BasicStroke strokeObjectOutline = new BasicStroke(1f);
			g2d.setXORMode(Color.white);
			g2d.setStroke(strokeObjectOutline);
			g2d.setColor(info.getColorFill());
			g2d.fill(shpShp);
			g2d.setColor(info.getColorBorder());
			g2d.draw(shpShp);
		}
		bRendering = false;
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
		pose = (Pose2D) listPoses.get(currentStep);
		shapeCurrent = (Shape) listShapes.get(currentStep);
	}
}
