package br.com.r4j.gui;

import java.awt.geom.Point2D;


/** @modelguid {F5DD3F6B-56F0-4129-8A13-824474278135} */
public class CanvasSelectionEvent
{
	/** @modelguid {51A20CE8-8CD2-4E29-A305-2F6666BA415A} */
	private Object imgMgr = null;
	/** @modelguid {5366C724-B096-4E5F-9DDD-4614A5BFDB31} */
	private Point2D pt1 = null;
	/** @modelguid {2834ABAA-7318-48BC-903F-2EEA6DB87C19} */
	private Point2D pt2 = null;
	/** @modelguid {53222574-FB9B-46D0-BE85-90BB30C14A2F} */
	private int button = 1;


	/** @modelguid {8ADE7A93-A1F2-425D-8013-7425E82A20C8} */
	public CanvasSelectionEvent(Object imgMgr, Point2D pt1, Point2D pt2, int button)
	{
		this.imgMgr = imgMgr;
		this.pt1 = pt1;
		this.pt2 = pt2;
		this.button = button;
	}


	/** @modelguid {9C657569-8A06-4528-AFAB-5EDF9203221D} */
	public Object getSource()
	{
		return imgMgr;
	}


	/** @modelguid {87B19373-0EAE-4CDC-A948-BB01F1193EDB} */
	public Point2D getWindowBegin()
	{
		return pt1;
	}


	/** @modelguid {1D7F379B-AD49-4E05-9FE8-62559BB0632D} */
	public Point2D getWindowEnd()
	{
		return pt2;
	}


	/** @modelguid {BE20F2EC-3FBD-426C-94ED-F56A9611DA2D} */
	public int getButton()
	{
		return button;
	}
}

