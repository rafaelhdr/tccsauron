package br.com.r4j.robosim;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.Iterator;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.robosim.estimator.Estimator;
import br.com.r4j.robosim.estimator.ParticleFilter;


/** @modelguid {21460D9B-249A-42BD-B521-C007281768F3} */
public class RobotParticles implements EstimatorRenderer, RendererListener
{
	/** @modelguid {36E6C213-76C4-45EC-BAF7-33C0DBD3C34C} */
	private static Log log = LogFactory.getLog(RobotParticles.class.getName());

	/** @modelguid {F2E06FC3-34D2-4C5A-AE84-77EA5EAD52B8} */
	private ArrayList listListParticles = null;
	/** @modelguid {16401DEE-D4D9-4BC2-AB10-C141494E949E} */
	private ArrayList particlesLast = null;
	/** @modelguid {69E4E784-CF21-40B1-9026-5F7E762E721B} */
	private ArrayList particlesCurrent = null;

	/** @modelguid {0DBBC172-AE58-46B4-8B8C-DBD0F60F85A8} */
	private Shape shpParticle = null;
	/** @modelguid {2C2088DA-11B0-4C94-B9DE-CBEB52234777} */
	private String strNameMyName = null;

	/** @modelguid {2E960B49-11D7-4A2B-942C-522BC3BF37D0} */
	private EstimatorRendererInfo info = null;
	/** @modelguid {7E0D57EC-5389-4C05-9F45-CC16101338D1} */
	private Estimator est = null;
	/** @modelguid {2269340F-3A73-4A79-9FF3-EA05DBBE9F3C} */
	private ParticleFilter pf = null;

	/** @modelguid {3DA8BB97-B8EA-40E0-A7E3-EF230C0C7C02} */
	private int currentStep = 0;
	

	/** @modelguid {B76AC532-DEBE-4AEB-B0F2-EC129C983329} */
	public RobotParticles(Estimator est)
	{
		this.strNameMyName = "Partículas";
		this.est = est;
		this.pf = (ParticleFilter) est;
		this.listListParticles = new ArrayList();
		this.shpParticle = ShapesUtil.createTriangle(Math.PI/3, 5, Math.PI/3, true);
	}

	/** @modelguid {CF100487-B6AE-40AB-8CCE-4AEF81B4C1B4} */
	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
		this.info = info;
	}


	/** @modelguid {93C93635-22B0-4E0E-9176-12885A04F582} */
	private boolean bRendering = false;
	/** @modelguid {F3473776-76E9-4EF4-A1A7-61989DC5A429} */
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


	/** @modelguid {9BDF1CE9-AE5D-4B75-8629-ECFDCAE5D82E} */
	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	/** @modelguid {8CF4029B-D5E0-4CA5-9A55-0044E5EAF5C3} */
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


	/** @modelguid {04A41376-2936-4CE3-BAA6-3320012EE04D} */
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


	/** @modelguid {E84BE34F-EC44-4411-9340-DF6F52C1BDF9} */
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


	/** @modelguid {7643E6F8-DADC-452A-829A-86FC03AC8C71} */
	public String getName()
	{
		return strNameMyName + " do " + est.getName();
	}


	/** @modelguid {60F86A9A-9D95-4051-98F3-B21F0F8259D6} */
	public void setName(String a)
	{
		strNameMyName = a;
	}


	/** @modelguid {BC3CC438-A84E-45A5-BDF9-F7CD3A15E435} */
	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
		particlesCurrent = (ArrayList) listListParticles.get(currentStep);
	}
}
