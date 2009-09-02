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
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class EstimatorRendererInfo implements Comparable
{
	private Shape shp = null;
	private Color colorFill = null;
	private Color colorBorder = null;
	
	private String strName = null;

	/**
	 * 
	 */
	public EstimatorRendererInfo(String a)
	{
		strName = a;
	}


	public String getName()
	{
		return strName;
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


	public int compareTo(Object o)
	{
		return strName.compareToIgnoreCase(((EstimatorRendererInfo) o).getName());
	}


	public EstimatorRendererInfo getCopy(String newName)
	{
		EstimatorRendererInfo info = new EstimatorRendererInfo(newName);
		info.setColorFill(this.getColorFill());
		info.setColorBorder(this.getColorBorder());
		info.setShape(this.getShape());

		return info;
	}
}
