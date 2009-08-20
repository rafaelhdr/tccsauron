package br.com.r4j.gui;

import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


public class RendererEvent
{
	private static Log log = LogFactory.getLog(RendererEvent.class.getName());

	private Object imgMgr = null;
	private Graphics2D g2d = null;
	private Graphics2D g2dUntransformed = null;
	private AffineTransform aff = null;
	private AffineTransform affInv = null;

	private AffineTransform affGr = null;
	private AffineTransform affGrInv = null;


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


	public Object getSource()
	{
		return imgMgr;
	}


	public Graphics2D getGraphics()
	{
		return g2d;
	}


	public Graphics2D getUntransformedGraphics()
	{
		return g2dUntransformed;
	}


	/**
	 * @param shapeCurrent
	 * @return
	 */
	public Shape getUnscaledShape(Shape shapeCurrent)
	{
		return affInv.createTransformedShape(shapeCurrent);
	}


	public void untransform()
	{
		g2d.setTransform(affGrInv);
	}


	public void transform()
	{
		g2d.setTransform(affGr);
	}


	public Shape untransform(Shape shp)
	{
		return affGrInv.createTransformedShape(shp);
	}


	public Shape transform(Shape shp)
	{
		return affGr.createTransformedShape(shp);
	}


	/**
	 * @param d
	 * @param e
	 * @param shpCircle
	 * @return
	 */
	public Shape translateAndMaintainShapeSize(double dx, double dy, Shape shp)
	{
		AffineTransform trafoTmp = AffineTransform.getTranslateInstance((aff.getTranslateX()/aff.getScaleX() + dx), (aff.getTranslateY()/aff.getScaleY() + dy));
		trafoTmp.concatenate(affInv);
		return trafoTmp.createTransformedShape(shp);
	}


	public Shape translate(double dx, double dy, Shape shp)
	{
		AffineTransform trafoTmp = AffineTransform.getTranslateInstance(dx, dy);
		return trafoTmp.createTransformedShape(shp);
	}
}

