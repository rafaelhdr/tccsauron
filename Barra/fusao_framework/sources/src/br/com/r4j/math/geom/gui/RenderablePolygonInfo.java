package br.com.r4j.math.geom.gui;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.List;
import java.util.*;
import javax.swing.*;
import javax.swing.event.*;
import br.com.r4j.math.*;
import br.com.r4j.math.geom.*;
import br.com.r4j.gui.*;


public class RenderablePolygonInfo
{
	private boolean bOutline = false;
	private boolean bFill = false;
	private Color colorFill = null;
	private Color colorOutline = null;
	private Stroke strokeObjectOutline = null;
	private ShapePolygon shpPolygon = null;


	public RenderablePolygonInfo(boolean anOutline, boolean aFill, Color aColorFill, Color aColorOutline, Stroke aStrokeObjectOutline, IPolygon aPoly)
	{
		bOutline = anOutline;
		bFill = aFill;
		colorFill = aColorFill;
		colorOutline = aColorOutline;
		strokeObjectOutline = aStrokeObjectOutline;
		shpPolygon = new ShapePolygon(aPoly.polygonIterator());
	}


	public void draw(Graphics2D gr2D, AffineTransform affTrns)
	{
		if (bFill)
		{
			gr2D.setColor(colorFill);
			gr2D.fill(shpPolygon);
		}
		if (bOutline)
		{
			gr2D.setColor(colorOutline);
			gr2D.setStroke(strokeObjectOutline);
			gr2D.draw(shpPolygon);
		}
	}


	public void draw(Graphics2D gr2D)
	{
		if (bFill)
		{
			gr2D.setColor(colorFill);
			gr2D.fill(shpPolygon);
		}
		if (bOutline)
		{
			gr2D.setColor(colorOutline);
			gr2D.setStroke(strokeObjectOutline);
			gr2D.draw(shpPolygon);
		}
	}


	public IPolygon getPolygon()
	{
		return shpPolygon;
	}


	public void setFillColor(Color clr)
	{
		colorFill = clr;
	}


	public Color getFillColor()
	{
		return colorFill;
	}

	public Rectangle getBounds()
	{
		return shpPolygon.getBounds();
	}

	public String toString()
	{
		return shpPolygon.toString();
	}
}