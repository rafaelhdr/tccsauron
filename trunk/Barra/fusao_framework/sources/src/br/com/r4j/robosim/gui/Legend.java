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
import java.util.Iterator;
import java.util.Map;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JToolTip;
import javax.swing.SwingUtilities;
import javax.swing.plaf.metal.MetalLabelUI;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.robosim.EstimatorRendererInfo;


public class Legend extends JLabel implements OnlineUI
{
	private static Log log = LogFactory.getLog(Legend.class.getName());

	private Map mapRendererInfos = null;
	private LegendUI ui = null;

	private String strTitle = "Legenda";
	

	public Legend()
	{
		ui = new LegendUI(this);
		this.setUI(ui);
	}

	public void setTitle(String a)	{strTitle = a;}
	public String getTitle()	{return strTitle;}


	public void setBackgroundColor(Color clr)
	{
		ui.setBackgroundColor(clr);
	}


	public void setBorderColor(Color clr)
	{
		ui.setBorderColor(clr);
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

	private Color clrBackground = null;
	private Color clrBorder = null;
	

	private String [] strs = null;
	private int maxWidth = 0;
	private JPanel pnl = null;
	private JToolTip toolTip = null;


	public LegendUI(Legend leg)
	{
		this.leg = leg;
		bCalc = false;

		clrBackground = new Color(254, 241, 169, 150);
		clrBorder = new Color(235, 182, 84);
	}


	public void setBackgroundColor(Color clr)
	{
		this.clrBackground = new Color(clr.getRed(), clr.getGreen(), clr.getBlue(), 150);
	}


	public void setBorderColor(Color clr)
	{
		this.clrBorder = clr;
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
			{
				int tmp = SwingUtilities.computeStringWidth(metrics, leg.getTitle()) + 100;
				if (tmp > width)
					width = tmp;
			}
			width += 16 + 4;
			height = 28 + (4 + metrics.getHeight())*(mapRendererInfos.size() + 1);
			log.debug("metrics.getHeight() = " + metrics.getHeight());

			buffImg = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
			Graphics2D g2d = (Graphics2D) buffImg.createGraphics();

			g2d.setPaint(clrBackground);
			g2d.fillRect(0, 0, width, height);
			g2d.setColor(clrBorder);
			g2d.drawRect(0, 0, width - 1, height - 1);

			g2d.setFont(fnt);
			g2d.setColor(clrBorder);
			String strLegend = leg.getTitle();
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
		if (buffImg != null)
			g.drawImage(buffImg, 0, 0, null);
		else
		{
			g.setColor(Color.red);
			g.fillRect(0, 0, 100, 100);
		}
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
;	public Dimension getPreferredSize(JComponent c)
	{
		return new Dimension(width, height);
	}
}  
