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


/** @modelguid {C7DD31E3-59EB-47F8-ADA3-1EDBE67D9CC3} */
public class RobotCovariance implements EstimatorRenderer, RendererListener
{
	/** @modelguid {C2A52B31-5D6A-469D-896A-67BDE8389C80} */
	private static Log log = LogFactory.getLog(RobotCovariance.class.getName());

	/** @modelguid {DE3216FB-F538-48E5-9F7A-512F07B3B226} */
	private ArrayList listPoses = null;
	/** @modelguid {8A234A76-0CC1-4AE5-BDDE-BC862CDB9E23} */
	private ArrayList listShapes = null;
	private ArrayList listPriAxis = null;
	private ArrayList listSecAxis = null;

	/** @modelguid {0DA56819-ACF5-41BA-8EBA-7264A445B577} */
	private Pose2D pose = null;
	/** @modelguid {4715FD20-ADBF-4E3A-91F8-AD5D060F2004} */
	private Pose2D poseLast = null;
	
	/** @modelguid {131892F5-F6B2-44F3-92B9-A7FB8EBE363D} */
	private Shape shapeLast = null;
	/** @modelguid {5177B282-D7B1-4291-8918-2407BED42D15} */
	private Shape shapeCurrent = null;
	private double aCurr = 0;
	private double bCurr = 0;

	/** @modelguid {13602F6B-4BD4-49A6-980B-36CDB5F106A4} */
	private String strNameMyName = null;

	/** @modelguid {F0A4B4D2-21F9-492E-AD0C-822BACF1D90E} */
	private EstimatorRendererInfo info = null;
	/** @modelguid {C1EEE7AE-0C74-47FE-8060-C0C4B5C35FA4} */
	private Estimator est = null;

	/** @modelguid {1AA85E67-1982-4572-9655-F4F2D46687CA} */
	private double c = 0;

	/** @modelguid {F5AE73DE-0F61-4EEE-B865-A06A2788092B} */
	private int currentStep = 0;
	

	/** @modelguid {9EC9CE39-99B4-4D77-A84F-6755708FF92B} */
	public RobotCovariance(Estimator est)
	{
		strNameMyName = "Covar Tracker";
		this.est = est;

		listPoses = new ArrayList();
		listShapes = new ArrayList();
		listPriAxis = new ArrayList();
		listSecAxis = new ArrayList();

		ChiSqrDistribution dist = new ChiSqrDistribution(2);
		c = dist.inverse(0.7);
		log.debug("dist.inverse(accProb) = " + c + ", dist.inverse(1- accProb) = " + dist.inverse(1 - 0.7));
	}

	/** @modelguid {E4E38630-657F-4D34-BF91-5F7DDF25F5D1} */
	private GradientPaint gradPaint = null;
	/** @modelguid {D6F5397D-4BB0-47F0-82D0-549A3882B51C} */
	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
		this.info = info;
		Color clr = info.getColorFill().brighter();
		clr = new Color(clr.getRed(), clr.getGreen(), clr.getBlue(), 150);
		Color clr2 = clr.brighter().brighter();
		gradPaint = new GradientPaint(0, 0, clr, 1000, 1000, clr2, true); 
	}


	/** @modelguid {76728614-1271-4B3F-ACDD-A09D9AC35E90} */
	private boolean bRendering = false;
	/** @modelguid {D2106B6B-64E6-4B99-AD8C-FCA183FE8D8E} */
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
//			log.debug("newPose: " + pose);
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

			Shape ellipse = ShapesUtil.createOvalTargetSingleCircle(A, B, 40);		
// 			Ellipse2D.Double ellipse = new Ellipse2D.Double(-A, -B, 2*A, 2*B);
			shapeCurrent = ShapesUtil.rotateShape(ellipse, eigenTheta);
			listPoses.add(pose);
			listShapes.add(shapeCurrent);
			listPriAxis.add(new Double(A));
			listSecAxis.add(new Double(B));
		}
		catch (MaximumIterationsExceededException e1)
		{
			log.error("error", e1);

			Ellipse2D.Double ellipse = new Ellipse2D.Double(-1, -1, 2*1, 2*1);
			shapeCurrent = ShapesUtil.rotateShape(ellipse, 0);
			listPoses.add(pose);
			listShapes.add(shapeCurrent);
			listPriAxis.add(new Double(1));
			listSecAxis.add(new Double(1));

		}
	}


	/** @modelguid {1EA138B6-240B-469B-84EB-51F8E097E580} */
	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	/** @modelguid {EEDC04B2-7029-471A-A503-2B8BCDAFAAD3} */
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


	/** @modelguid {57ED4680-32C5-4E34-88C4-04ED62F16427} */
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


	/** @modelguid {3E0AA9F0-992F-4871-9904-07D33B5B5057} */
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


	/** @modelguid {AA4F09F5-E13D-495C-9DAD-D9F5275A78F6} */
	public String getName()
	{
		return strNameMyName;
	}


	/** @modelguid {E0975D40-7B6D-43D2-996C-9185194F82C5} */
	public void setName(String a)
	{
		strNameMyName = a;
	}


	/** @modelguid {946F36DA-6292-488E-BE25-456A037D7F00} */
	public void setStep(int currentStep)
	{
		this.currentStep = currentStep;
		pose = (Pose2D) listPoses.get(currentStep);
		shapeCurrent = (Shape) listShapes.get(currentStep);
		aCurr = ((Double) listPriAxis.get(currentStep)).doubleValue();
		bCurr = ((Double) listSecAxis.get(currentStep)).doubleValue();
	}
}
