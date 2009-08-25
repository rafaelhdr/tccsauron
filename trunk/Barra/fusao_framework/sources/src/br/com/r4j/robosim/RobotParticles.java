package br.com.r4j.robosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.Point;
import java.awt.geom.Ellipse2D;
import java.util.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.LinearMath;
import JSci.maths.MaximumIterationsExceededException;
import JSci.maths.statistics.ChiSqrDistribution;
import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.robosim.estimator.*;


public class RobotParticles implements EstimatorRenderer, RendererListener
{
	private static Log log = LogFactory.getLog(RobotParticles.class.getName());

	private ArrayList listListParticles = null;
	private ArrayList particlesLast = null;
	private ArrayList particlesCurrent = null;

	private Shape shpParticle = null;
	private String strNameMyName = null;

	private EstimatorRendererInfo info = null;
	private Estimator est = null;
	private ParticleFilter pf = null;

	private int currentStep = 0;
	

	public RobotParticles(Estimator est)
	{
		this.strNameMyName = "Part�culas";
		this.est = est;
		this.pf = (ParticleFilter) est;
		this.listListParticles = new ArrayList();
		this.shpParticle = ShapesUtil.createTriangle(Math.PI/3, 5, Math.PI/3, true);
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
		ArrayList listParticles = new ArrayList();
		listListParticles.add(listParticles);
		AbstractDoubleMatrix state = pf.getParticles();

		for (int i = 0; i < state.rows(); i++) 
		{
			double xP = state.getElement(i, 0);
			double yP = state.getElement(i, 1);
			listParticles.add(new Point((int) xP, (int) yP));
		}
		particlesCurrent = listParticles;
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	public void updatePerformed(RendererEvent e)
	{
		bRendering = true;
		if (particlesCurrent != null)
		{
			Graphics2D g2d = e.getGraphics();
			g2d.setPaintMode();
			g2d.setColor(info.getColorFill());
			Iterator itParticles = particlesCurrent.iterator();
			while (itParticles.hasNext())
			{
				Point ptPf = (Point) itParticles.next();
				Shape shpShp = e.translateAndMaintainShapeSize(ptPf.x, ptPf.y, shpParticle);
				g2d.fill(shpShp);
			}
			particlesLast = particlesCurrent;
		}
		bRendering = false;
	}


	public void render(RendererEvent e)
	{
		bRendering = true;
		if (particlesCurrent != null)
		{
			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);
			g2d.setColor(info.getColorFill());
			Iterator itParticles = particlesCurrent.iterator();
			while (itParticles.hasNext())
			{
				Point ptPf = (Point) itParticles.next();
				Shape shpShp = e.translateAndMaintainShapeSize(ptPf.x, ptPf.y, shpParticle);
				g2d.fill(shpShp);
			}
			particlesLast = particlesCurrent;
		}
		bRendering = false;
	}


	public void erase(RendererEvent e)
	{
		bRendering = true;
		if (particlesLast != null)
		{
			Graphics2D g2d = e.getGraphics();
			g2d.setXORMode(Color.white);
			g2d.setColor(info.getColorFill());
			Iterator itParticles = particlesLast.iterator();
			while (itParticles.hasNext())
			{
				Point ptPf = (Point) itParticles.next();
				Shape shpShp = e.translateAndMaintainShapeSize(ptPf.x, ptPf.y, shpParticle);
				g2d.fill(shpShp);
			}
		}
		bRendering = false;
	}


	public String getName()
	{
		return strNameMyName + " do " + est.getName();
	}


	public void setName(String a)
	{
		strNameMyName = a;
	}


	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
		particlesCurrent = (ArrayList) listListParticles.get(currentStep);
	}
}