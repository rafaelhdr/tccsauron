package br.com.r4j.gui;

import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/** @modelguid {ED2B7EFF-BA8F-4F2A-83F7-F4A4C51A0A35} */
public class RendererEvent
{
	/** @modelguid {00F82FB3-DE19-4665-B933-73965D2B38E3} */
	private static Log log = LogFactory.getLog(RendererEvent.class.getName());

	/** @modelguid {EE836F16-4310-4D0C-8215-1D267E7DFA54} */
	private Object imgMgr = null;
	/** @modelguid {F1553A49-2E36-401E-A61B-55F7ECF8B71E} */
	private Graphics2D g2d = null;
	/** @modelguid {4D9A763C-A957-4426-877C-E6C658ECC294} */
	private Graphics2D g2dUntransformed = null;
	/** @modelguid {ECE06C3D-0349-45F3-AAAD-FF3C8BE1F27A} */
	private AffineTransform aff = null;
	/** @modelguid {6CD5AE25-9AE5-4124-8327-415D01592E2E} */
	private AffineTransform affInv = null;

	/** @modelguid {2DC64590-A790-478A-9DA6-19E26D0C9702} */
	private AffineTransform affGr = null;
	/** @modelguid {8753E37C-8AA7-425A-B825-51506C89A272} */
	private AffineTransform affGrInv = null;


	/** @modelguid {24463EE6-E383-4044-AAF4-066EE4882647} */
	public RendererEvent(Object imgMgr, Graphics2D g2d, Graphics2D g2dUntransformed, AffineTransform aff)
	{
		this.imgMgr = imgMgr;
		this.g2d = g2d;
		this.g2dUntransformed = g2dUntransformed; 
		this.aff = aff;
		
		this.affGr = g2d.getTransform(); 
		
		try
		{
			this.affGrInv = affGr.createInverse(); 
			this.affInv = aff.createInverse();
		}
		catch (NoninvertibleTransformException e)
		{
			log.error("RendererEvent", e);
			e.printStackTrace();
		}
	}


	/** @modelguid {F43C4BA2-2F97-4E20-9B2D-69F440F569A6} */
	public RendererEvent(Object imgMgr, Graphics2D g2d, Graphics2D g2dUntransformed, AffineTransform aff, AffineTransform affInv)
	{
		this.imgMgr = imgMgr;
		this.g2d = g2d;
		this.g2dUntransformed = g2dUntransformed; 
		this.aff = aff;
		this.affInv = affInv;

		this.affGr = g2d.getTransform(); 
		try
		{
			this.affGrInv = affGr.createInverse(); 
		}
		catch (NoninvertibleTransformException e)
		{
			log.error("RendererEvent", e);
			e.printStackTrace();
		}
	}


	/** @modelguid {BA17F54B-04C0-4F96-BE35-78044E6202D8} */
	public Object getSource()
	{
		return imgMgr;
	}


	/** @modelguid {1B9580BD-2596-42BD-BAF9-FAE5AF8FCC11} */
	public Graphics2D getGraphics()
	{
		return g2d;
	}


	/** @modelguid {692659D2-52D8-4A61-BC65-B2391DEA9547} */
	public Graphics2D getUntransformedGraphics()
	{
		return g2dUntransformed;
	}


	/**
	 * @param shapeCurrent
	 * @return
	 * @modelguid {2851D023-6717-4BE3-9E11-668F269BBBE5}
	 */
	public Shape getUnscaledShape(Shape shapeCurrent)
	{
		return affInv.createTransformedShape(shapeCurrent);
	}


	/** @modelguid {E8BDC220-8CA1-4871-B050-13CE86767DE2} */
	public void untransform()
	{
		g2d.setTransform(affGrInv);
	}


	/** @modelguid {68794BA1-20CF-49DE-811F-4323ECBE7043} */
	public void transform()
	{
		g2d.setTransform(affGr);
	}


	/** @modelguid {04DBE499-91F4-477A-93C9-E25CDA3CFAC1} */
	public Shape untransform(Shape shp)
	{
		return affGrInv.createTransformedShape(shp);
	}


	/** @modelguid {B39B9BE8-8E7D-4120-B00B-97A622AE6F1A} */
	public Shape transform(Shape shp)
	{
		return affGr.createTransformedShape(shp);
	}


	/**
	 * @param d
	 * @param e
	 * @param shpCircle
	 * @return
	 * @modelguid {7BD14B29-805B-4ECF-942F-28F201CA83DE}
	 */
	public Shape translateAndMaintainShapeSize(double dx, double dy, Shape shp)
	{
		AffineTransform trafoTmp = AffineTransform.getTranslateInstance((aff.getTranslateX()/aff.getScaleX() + dx), (aff.getTranslateY()/aff.getScaleY() + dy));
		trafoTmp.concatenate(affInv);
		return trafoTmp.createTransformedShape(shp);
	}


	/** @modelguid {652AAE4D-6009-434C-94B8-817F7DF9BF17} */
	public Shape translate(double dx, double dy, Shape shp)
	{
		AffineTransform trafoTmp = AffineTransform.getTranslateInstance(dx, dy);
		return trafoTmp.createTransformedShape(shp);
	}
}

