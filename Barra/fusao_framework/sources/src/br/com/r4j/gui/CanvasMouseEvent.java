package br.com.r4j.gui;

import java.awt.geom.Point2D;


/** @modelguid {77612460-512B-429A-9876-5A3B6891F2C0} */
public class CanvasMouseEvent
{
	/** @modelguid {E1E4CC9F-C444-41F8-98A8-34D9B4D23C5C} */
	private Object imgMgr = null;
	/** @modelguid {19948C1E-DB4F-4433-84D6-881DCF9A595D} */
	private Point2D pt = null;
	/** @modelguid {164CF94B-B3F2-4BCD-8E90-8DCAA5F956D0} */
	private int button = 1;


	/** @modelguid {795E9F92-1752-478C-8284-297E0CFE712D} */
	public CanvasMouseEvent(Object imgMgr, Point2D pt, int button)
	{
		this.imgMgr = imgMgr;
		this.pt = pt;
		this.button = button;
	}


	/** @modelguid {6DE1849F-3DB5-4D25-AC11-993F913F30C8} */
	public Object getSource()
	{
		return imgMgr;
	}


	/** @modelguid {27767770-A689-44C6-8EDB-8613F7D42F30} */
	public Point2D getPixelLocation()
	{
		return pt;
	}


	/** @modelguid {AFDD7E5F-27B6-4B7E-A046-419EFF7112EB} */
	public int getButton()
	{
		return button;
	}
}

