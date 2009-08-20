package br.com.r4j.robosim.gui;

import javax.swing.plaf.metal.*;
import javax.swing.plaf.*;
import javax.accessibility.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.awt.*;
import java.util.*;
import java.io.*;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.robosim.EstimatorRendererInfo;


public class Legend extends JLabel implements OnlineUI
{
	private static Log log = LogFactory.getLog(Legend.class.getName());

	private Map mapRendererInfos = null;
	private LegendUI ui = null;


	public Legend()
	{
		ui = new LegendUI(this);
		this.setUI(ui);
	}


	public void setRendererInfos(Map mapRendererInfos)
	{
		this.mapRendererInfos = mapRendererInfos;
	}


	public Map getRendererInfos()
	{
		return mapRendererInfos;
	}


	public void resetUI()
	{
		ui.reset();
	}
	
	
	public JComponent getComponent()
	{
		return this;
	}
}


class LegendUI extends MetalLabelUI
{
	private static Log log = LogFactory.getLog(LegendUI.class.getName());

	private Legend leg = null;

	private boolean bCalc = false;

	private BufferedImage buffImg = null;
	private  int width = 1;
	private  int height = 1;
	

	private String [] strs = null;
	private int maxWidth = 0;
	private JPanel pnl = null;
	private JToolTip toolTip = null;


	public LegendUI(Legend leg)
	{
		this.leg = leg;
		bCalc = false;
	}

	
	public void reset()
	{
		bCalc = false;
	}


	private void calc()
	{
		bCalc = true;
		Map mapRendererInfos = leg.getRendererInfos();
		
		if (mapRendererInfos != null)
		{
			Font fnt = new Font("Courier", Font.BOLD, 12);
			FontMetrics metrics = Toolkit.getDefaultToolkit().getFontMetrics(fnt);

			Iterator itInfos = mapRendererInfos.keySet().iterator();
			width = 0;
			while (itInfos.hasNext())
			{
				EstimatorRendererInfo rndrInfo = (EstimatorRendererInfo) itInfos.next();
				String strName = (String) mapRendererInfos.get(rndrInfo);
				int tmp = SwingUtilities.computeStringWidth(metrics, strName);
				if (tmp > width)
					width = tmp;
			}
			width += 16 + 4;
			height = 28 + (4 + metrics.getHeight())*(mapRendererInfos.size() + 1);
			log.debug("metrics.getHeight() = " + metrics.getHeight());

			buffImg = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
			Graphics2D g2d = (Graphics2D) buffImg.createGraphics();

			g2d.setPaint(new Color(254, 241, 169, 150));
			g2d.fillRect(0, 0, width, height);
			g2d.setColor(new Color(235, 182, 84));
			g2d.drawRect(0, 0, width - 1, height - 1);

			g2d.setFont(fnt);
			g2d.setColor(new Color(235, 182, 84));
			String strLegend = "Legenda";
			int widthStr = SwingUtilities.computeStringWidth(metrics, strLegend);
			g2d.drawString(strLegend, 50 - widthStr/2, metrics.getHeight() + 2);
			g2d.drawLine(3, metrics.getHeight() + 6, width - 4, metrics.getHeight() + 6);

			Font fnt2 = new Font("Courier", Font.PLAIN, 12);
			FontMetrics metrics2 = Toolkit.getDefaultToolkit().getFontMetrics(fnt2);

			int heightIt = metrics.getHeight() + 10;
			itInfos = mapRendererInfos.keySet().iterator();
			while (itInfos.hasNext())
			{
				EstimatorRendererInfo rndrInfo = (EstimatorRendererInfo) itInfos.next();
				String strName = (String) mapRendererInfos.get(rndrInfo);
				
				heightIt += 4 + metrics2.getHeight(); 
				int heightItUp = heightIt - metrics2.getHeight(); 
				
				BasicStroke strokeObjectOutline = new BasicStroke(2f);
				g2d.setStroke(strokeObjectOutline);
				g2d.setPaint(rndrInfo.getColorFill());
				g2d.fillRect(3, heightItUp, 10, 10);
				g2d.setColor(rndrInfo.getColorBorder());
				g2d.drawRect(3, heightItUp, 10, 10);

				g2d.setColor(new Color(235, 182, 84));
				g2d.drawString(strName, 17, heightItUp + 10);
			}
		}
		leg.setSize(width, height);
	}


	public void paint(Graphics g, JComponent c)
	{
		if (!bCalc)
			calc();
		g.drawImage(buffImg, 0, 0, null);
		leg.setSize(width, height);
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
