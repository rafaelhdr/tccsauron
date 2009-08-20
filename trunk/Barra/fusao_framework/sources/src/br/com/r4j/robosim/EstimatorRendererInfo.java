/*
 * Created on Dec 1, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim;

import java.awt.Color;
import java.awt.Shape;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class EstimatorRendererInfo
{
	private Shape shp = null;
	private Color colorFill = null;
	private Color colorBorder = null;
	

	/**
	 * 
	 */
	public EstimatorRendererInfo()
	{
		super();
	}


	public Color getColorFill()
	{
		return colorFill;
	}


	public void setColorFill(Color color)
	{
		this.colorFill = color;
	}


	public Color getColorBorder()
	{
		return colorBorder;
	}


	public void setColorBorder(Color color)
	{
		this.colorBorder = color;
	}


	public void setShape(Shape shape)
	{
		this.shp = shape;
	}


	public Shape getShape()
	{
		return shp;
	}
}
