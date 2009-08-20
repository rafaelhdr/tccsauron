package br.com.r4j.gui;

import java.awt.geom.Point2D;


public class CanvasMouseEvent
{
	private Object imgMgr = null;
	private Point2D pt = null;
	private int button = 1;


	public CanvasMouseEvent(Object imgMgr, Point2D pt, int button)
	{
		this.imgMgr = imgMgr;
		this.pt = pt;
		this.button = button;
	}


	public Object getSource()
	{
		return imgMgr;
	}


	public Point2D getPixelLocation()
	{
		return pt;
	}


	public int getButton()
	{
		return button;
	}
}

