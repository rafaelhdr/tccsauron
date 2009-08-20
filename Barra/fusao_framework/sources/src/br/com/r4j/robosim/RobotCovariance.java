package br.com.r4j.robosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.LinearMath;
import JSci.maths.MaximumIterationsExceededException;
import JSci.maths.statistics.ChiSqrDistribution;
import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.robosim.estimator.Estimator;


public class RobotCovariance implements EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(RobotCovariance.class.getName());

	private ArrayList listPoses = null;
	private ArrayList listShapes = null;

	private Pose2D pose = null;
	private Pose2D poseLast = null;
	
	private Shape shapeLast = null;
	private Shape shapeCurrent = null;

	private String strNameMyName = null;

	private EstimatorRendererInfo info = null;
	private Estimator est = null;

	private double c = 0;

	private int currentStep = 0;
	

	public RobotCovariance(Estimator est)
	{
		strNameMyName = "Covar Tracker";
		this.est = est;

		listPoses = new ArrayList();
		listShapes = new ArrayList();

		ChiSqrDistribution dist = new ChiSqrDistribution(2);
		c = dist.inverse(0.5);
		log.debug("dist.inverse(accProb) = " + c + ", dist.inverse(1- accProb) = " + dist.inverse(1 - 0.5));
	}

	private GradientPaint gradPaint = null;
	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
		this.info = info;
		Color clr = info.getColorFill().brighter();
		clr = new Color(clr.getRed(), clr.getGreen(), clr.getBlue(), 150);
		Color clr2 = clr.brighter().brighter();
		gradPaint = new GradientPaint(0, 0, clr, 1000, 1000, clr2, true); 
	}


	private boolean bRendering = false;
	public void newPose(Pose2D pose)
	{
		int countWait = 0; while (bRendering) 
		{
			if (countWait++ > 4) bRendering = false;
			else try {Thread.sleep(20);} catch (Exception e) {}
		}
		try
		{
			currentStep++;
			log.debug("newPose: " + pose);
			this.pose = pose;
		
			AbstractDoubleSquareMatrix covar = est.getCovariance();
			AbstractDoubleVector [] eigenvectors = new AbstractDoubleVector[2];
		
			double [] arrayEigenValues = LinearMath.eigenSolveSymmetric(MatrixUtil.subMatrix(covar, 0, 2), eigenvectors);
			double eigenX = eigenvectors[0].getComponent(0), eigenY = eigenvectors[0].getComponent(1);  
			double eigenTheta = 0; 
			if (eigenX*eigenX > 0.0025)
			{
				eigenTheta = Math.atan(eigenY/eigenX);
				if (eigenX < 0)
					eigenTheta += Math.PI;
			}
			else if (eigenY > 0)
				eigenTheta = Math.PI/2;
			else
				eigenTheta = 3*Math.PI/2;
				
			double A = Math.sqrt(arrayEigenValues[0])*c;
			double B = Math.sqrt(arrayEigenValues[1])*c;
			double theta1 = Math.atan2(eigenvectors[0].getComponent(1), eigenvectors[0].getComponent(0));
		
 			Ellipse2D.Double ellipse = new Ellipse2D.Double(-A, -B, 2*A, 2*B);
			shapeCurrent = ShapesUtil.rotateShape(ellipse, eigenTheta);
			listPoses.add(pose);
			listShapes.add(shapeCurrent);
		}
		catch (MaximumIterationsExceededException e1)
		{
			log.error("error", e1);

			Ellipse2D.Double ellipse = new Ellipse2D.Double(-1, -1, 2*1, 2*1);
			shapeCurrent = ShapesUtil.rotateShape(ellipse, 0);
			listPoses.add(pose);
			listShapes.add(shapeCurrent);

		}
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;
		if (pose != null)
		{
			Graphics2D g2d = e.getGraphics();
//				g2d.setXORMode(Color.white);
			g2d.setPaintMode();

			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
//			g2d.setPaint(gradPaint); 
			Shape shapeTmp = e.translate(pose.getX(), pose.getY(), shapeCurrent);
//			g2d.fill(shapeTmp);
			g2d.setColor(info.getColorFill());
			g2d.draw(shapeTmp);

			shapeLast = shapeTmp;
			poseLast = pose; 
		}
		bRendering = false;
	}


	public void render(RendererEvent e)
	{
		bRendering = true;
		if (pose != null)
		{
			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);

			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
//			g2d.setPaint(gradPaint); 
			Shape shapeTmp = e.translate(pose.getX(), pose.getY(), shapeCurrent);
//			g2d.fill(shapeTmp);
			g2d.setColor(info.getColorFill());
			g2d.draw(shapeTmp);

			shapeLast = shapeTmp;
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
			g2d.setXORMode(Color.white);
	
			BasicStroke strokeObjectOutline = new BasicStroke(4f);
			g2d.setStroke(strokeObjectOutline);
//			g2d.setPaint(gradPaint); 
//			g2d.fill(shapeLast);
			g2d.setColor(info.getColorFill());
			g2d.draw(shapeLast);
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
