package br.com.r4j.gui;

import java.awt.geom.Point2D;


public class CanvasSelectionEvent
{
	private Object imgMgr = null;
	private Point2D pt1 = null;
	private Point2D pt2 = null;
	private int button = 1;


	public CanvasSelectionEvent(Object imgMgr, Point2D pt1, Point2D pt2, int button)
	{
		this.imgMgr = imgMgr;
		this.pt1 = pt1;
		this.pt2 = pt2;
		this.button = button;
	}


	public Object getSource()
	{
		return imgMgr;
	}


	public Point2D getWindowBegin()
	{
		return pt1;
	}


	public Point2D getWindowEnd()
	{
		return pt2;
	}


	public int getButton()
	{
		return button;
	}
}

