package br.com.r4j.math.geom.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.Stroke;
import java.awt.geom.AffineTransform;

import br.com.r4j.math.geom.IPolygon;
import br.com.r4j.math.geom.ShapePolygon;


/** @modelguid {73568F0B-796F-468A-BA6D-19B1076BC0FB} */
public class RenderablePolygonInfo
{
	/** @modelguid {D1CA1EE0-FF3B-4CED-AF13-5CB099224B6C} */
	private boolean bOutline = false;
	/** @modelguid {35F77B26-33FC-4360-A1F0-9DE70CBA6E5E} */
	private boolean bFill = false;
	/** @modelguid {97D683B2-575B-45C0-BAF0-05E767816040} */
	private Color colorFill = null;
	/** @modelguid {34099DB6-B47E-4934-AD40-0C94727F6401} */
	private Color colorOutline = null;
	/** @modelguid {5EF483A3-CDDD-415B-A81C-DD7CA2B52565} */
	private Stroke strokeObjectOutline = null;
	/** @modelguid {D8E6F744-C308-47CF-8E46-240126FB617B} */
	private ShapePolygon shpPolygon = null;


	/** @modelguid {CDA2DEDE-6D95-4D41-9BE1-9CB4B515B5C7} */
	public RenderablePolygonInfo(boolean anOutline, boolean aFill, Color aColorFill, Color aColorOutline, Stroke aStrokeObjectOutline, IPolygon aPoly)
	{
		bOutline = anOutline;
		bFill = aFill;
		colorFill = aColorFill;
		colorOutline = aColorOutline;
		strokeObjectOutline = aStrokeObjectOutline;
		shpPolygon = new ShapePolygon(aPoly.polygonIterator());
	}


	/** @modelguid {49C44C90-7BBF-4228-AE8D-905A487E00E4} */
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


	/** @modelguid {4F3029E1-719F-44FA-903B-3FB09122845A} */
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


	/** @modelguid {2FDF49A4-0FEA-4821-A76B-2ADF9C04B865} */
	public IPolygon getPolygon()
	{
		return shpPolygon;
	}


	/** @modelguid {3B0A9983-5F94-4810-86DC-58C8DB71F6E9} */
	public void setFillColor(Color clr)
	{
		colorFill = clr;
	}


	/** @modelguid {ABB7C93F-76B6-4592-89A9-535C1507530E} */
	public Color getFillColor()
	{
		return colorFill;
	}

	/** @modelguid {7045C89B-00CE-4398-9334-DE7AF168478A} */
	public Rectangle getBounds()
	{
		return shpPolygon.getBounds();
	}

	/** @modelguid {9A535DF9-3404-4329-A7D6-1C0C693166A2} */
	public String toString()
	{
		return shpPolygon.toString();
	}
}