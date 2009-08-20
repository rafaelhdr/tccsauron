	/*
 * Created on Dec 12, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToolTip;
import javax.swing.SwingUtilities;
import javax.swing.plaf.metal.MetalLabelUI;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.robosim.EstimatorRenderer;
import br.com.r4j.robosim.EstimatorRendererInfo;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.RealTrackGenerator;
import br.com.r4j.robosim.RobotPlayer;
import br.com.r4j.robosim.estimator.Estimator;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class OnlineRMSError extends JLabel implements OnlineUI
{
	private static Log log = LogFactory.getLog(Legend.class.getName());

	private Map mapRendererInfos = null;
	private Map mapRndr2RndrrInfo = null;

	protected EstimatorRendererInfo [] arrayInfo = null; 
	protected ArrayList [] arrayListPoses = null; 
	protected double [] arrayRMS = null; 

	private List listTrakers = null;

	protected int currrentStep = 0;
	protected int updateCount = 0;
	protected boolean bNeedsUpdate = true;

	private OnlineRMSErrorUI ui = null;


	/**
	 * 
	 */
	public OnlineRMSError()
	{
		super();
		mapRendererInfos = new HashMap();
		mapRndr2RndrrInfo = new HashMap();
		listTrakers = new ArrayList();

		ui = new OnlineRMSErrorUI(this);
		this.setUI(ui);
	}

	/**
	 * @param robotPlayer
	 * @param realInfo
	 */
	public void setRealTrackGenerator(RealTrackGenerator robotPlayer, EstimatorRendererInfo realInfo, RobotPlayer rPlayer)
	{
		ImplEstimatorRenderer tmp = new ImplEstimatorRenderer(this, listTrakers.size());
		((RealTrackGenerator) robotPlayer).addRobotTracker(tmp);
		rPlayer.addEstimatorRenderer(tmp);
		mapRndr2RndrrInfo.put(robotPlayer, realInfo);
		listTrakers.add(robotPlayer);
	}

	/**
	 * @param est
	 * @param estInfo
	 */
	public void setEstimator(Estimator est, EstimatorRendererInfo estInfo, RobotPlayer rPlayer)
	{
		ImplEstimatorRenderer tmp = new ImplEstimatorRenderer(this, listTrakers.size());
		est.addRobotTracker(tmp);
		rPlayer.addEstimatorRenderer(tmp);
		mapRndr2RndrrInfo.put(est, estInfo);
		listTrakers.add(est);
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.gui.OnlineUI#setRendererInfos(java.util.Map)
	 */
	public void setRendererInfos(Map mapRendererInfos)
	{
		this.mapRendererInfos = mapRendererInfos;
	}


	public void newPose(Pose2D pose, int idx)
	{
		updateCount++;
		arrayListPoses[idx].add(pose);
		this.currrentStep = arrayListPoses[idx].size() - 1;
		if (updateCount == arrayListPoses.length)
		{
			updateCount = 0;
			this.computeValues(currrentStep);
		}
		bNeedsUpdate = true;  
	}


	/**
	 * 
	 */
	private void computeValues(int currentStepTmp)
	{
		Pose2D poseReal = (Pose2D) arrayListPoses[0].get(currentStepTmp);
		arrayRMS[0] = 0;
		for (int i = 1; i < arrayRMS.length; i++)
		{
			Pose2D poseEst = (Pose2D) arrayListPoses[i].get(currentStepTmp);
			double dx = poseEst.getX() - poseReal.getX();
			double dy = poseEst.getY() - poseReal.getY();
			arrayRMS[i] = Math.sqrt(dy*dy + dx*dx); 
		}
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.EstimatorRenderer#setStep(int)
	 */
	public void setStep(int currentStep, int idx)
	{
		updateCount++;
		this.currrentStep = currentStep;
		if (updateCount == arrayListPoses.length)
		{
			updateCount = 0;
			this.computeValues(currentStep);
		}
		bNeedsUpdate = true;  
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.gui.OnlineUI#getRendererInfos()
	 */
	public Map getRendererInfos()
	{
		return mapRendererInfos;
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.gui.OnlineUI#resetUI()
	 */
	public void resetUI()
	{
		arrayInfo = new EstimatorRendererInfo[listTrakers.size()]; 
		arrayListPoses = new ArrayList[listTrakers.size()];
		arrayRMS = new double[listTrakers.size()];
		for (int i = 0; i < listTrakers.size(); i++)
		{
			arrayInfo[i] = (EstimatorRendererInfo) mapRndr2RndrrInfo.get(listTrakers.get(i));
			arrayListPoses[i] = new ArrayList();
		} 
		ui.reset();
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.gui.OnlineUI#getUI()
	 */
	public JComponent getComponent()
	{
		return this;
	}

}

class ImplEstimatorRenderer implements EstimatorRenderer
{
	private OnlineRMSError callback = null;
	private int idxSource = 0;
	
	
	public ImplEstimatorRenderer(OnlineRMSError callback, int idxSource)
	{
		this.callback = callback;
		this.idxSource = idxSource;
	}


	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.EstimatorRenderer#newPose(br.com.r4j.robosim.Pose2D)
	 */
	public void newPose(Pose2D pose)
	{
		callback.newPose(pose, idxSource);
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.EstimatorRenderer#setEstimatorRendererInfo(br.com.r4j.robosim.EstimatorRendererInfo)
	 */
	public void setEstimatorRendererInfo(EstimatorRendererInfo info)
	{
	}

	/* (non-Javadoc)
	 * @see br.com.r4j.robosim.EstimatorRenderer#setStep(int)
	 */
	public void setStep(int currentStep)
	{
		callback.setStep(currentStep, idxSource);
	}
}

class OnlineRMSErrorUI extends MetalLabelUI
{
	private static Log log = LogFactory.getLog(OnlineRMSErrorUI.class.getName());

	private OnlineRMSError rmsErr = null;

	private BufferedImage buffImg = null;
	private  int width = 1;
	private  int height = 1;

	private String [] strs = null;
	private int maxWidth = 0;
	private JPanel pnl = null;
	private JToolTip toolTip = null;


	public OnlineRMSErrorUI(OnlineRMSError rmsErr)
	{
		this.rmsErr = rmsErr;
	}

	
	public void reset()
	{
	}


	private void calc()
	{
		try
		{
			int count = 0;
			while (rmsErr.updateCount != 0)
			{
				count++;
				Thread.sleep(10);
				if (count > 18)
				{
					log.debug("calc: if (count > 18)");
					return;
				}
			}
		}
		catch (Exception e)
		{
			log.debug("erro", e);
		}
		rmsErr.bNeedsUpdate = false;
		
		Map mapRendererInfos = rmsErr.getRendererInfos();
		
		if (mapRendererInfos != null)
		{
			Font fnt = new Font("Courier", Font.BOLD, 12);
			FontMetrics metrics = Toolkit.getDefaultToolkit().getFontMetrics(fnt);
			width = 0;
			for (int i = 1; i < rmsErr.arrayRMS.length; i++)
			{
				int tmp = SwingUtilities.computeStringWidth(metrics, "" + rmsErr.arrayRMS[i]);
				if (tmp > width)
					width = tmp;
			}
			{
				int tmp = SwingUtilities.computeStringWidth(metrics, "RMS") + 100;
				if (tmp > width)
					width = tmp;
			}
			
			width += 16 + 4;
			height = 28 + (4 + metrics.getHeight())*(rmsErr.arrayRMS.length);

			buffImg = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
			Graphics2D g2d = (Graphics2D) buffImg.createGraphics();

			g2d.setPaint(new Color(191, 191, 255, 150));
			g2d.fillRect(0, 0, width, height);
			g2d.setColor(new Color(128, 0, 255));
			g2d.drawRect(0, 0, width - 1, height - 1);

			g2d.setFont(fnt);
			g2d.setColor(new Color(128, 0, 255));
			String strLegend = "RMS";
			int widthStr = SwingUtilities.computeStringWidth(metrics, strLegend);
			g2d.drawString(strLegend, 50 - widthStr/2, metrics.getHeight() + 2);
			g2d.drawLine(3, metrics.getHeight() + 6, width - 4, metrics.getHeight() + 6);

			Font fnt2 = new Font("Courier", Font.PLAIN, 12);
			FontMetrics metrics2 = Toolkit.getDefaultToolkit().getFontMetrics(fnt2);

			int heightIt = metrics.getHeight() + 10;

			for (int i = 1; i < rmsErr.arrayInfo.length; i++)
			{			
				EstimatorRendererInfo rndrInfo = rmsErr.arrayInfo[i];

				heightIt += 4 + metrics2.getHeight(); 
				int heightItUp = heightIt - metrics2.getHeight(); 
				
				BasicStroke strokeObjectOutline = new BasicStroke(2f);
				g2d.setStroke(strokeObjectOutline);
				g2d.setPaint(rndrInfo.getColorFill());
				g2d.fillRect(3, heightItUp, 10, 10);
				g2d.setColor(rndrInfo.getColorBorder());
				g2d.drawRect(3, heightItUp, 10, 10);

				g2d.setColor(new Color(128, 0, 255));
				g2d.drawString(rmsErr.arrayRMS[i] + "", 17, heightItUp + 10);
			}

			rmsErr.setSize(width, height);
		}
	}


	public void paint(Graphics g, JComponent c)
	{
		if (rmsErr.bNeedsUpdate)
			this.calc();
			
		g.drawImage(buffImg, 0, 0, null);
		rmsErr.setSize(width, height);
	}


	public Dimension getMaximumSize(JComponent c)
	{
		return new Dimension(width, height);
	}
	public Dimension getMinimumSize(JComponent c)
	{
		return new Dimension(width, height);
	}
	public Dimension getPreferredSize(JComponent c)
	{
		return new Dimension(width, height);
	}
}  


