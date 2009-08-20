package br.com.r4j.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.TexturePaint;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.dnd.DnDConstants;
import java.awt.dnd.DropTarget;
import java.awt.dnd.DropTargetDragEvent;
import java.awt.dnd.DropTargetDropEvent;
import java.awt.dnd.DropTargetEvent;
import java.awt.dnd.DropTargetListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.image.BufferedImage;
import java.awt.image.SampleModel;
import java.awt.image.SinglePixelPackedSampleModel;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.swing.JComponent;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JViewport;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.EventListenerList;
import javax.swing.event.MouseInputListener;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.commons.util.ObjectHolder;
import br.com.r4j.commons.util.TraceUtil;


/**
 *  Classe que da suporte a:
 *  - operações de zoom e reposicionamento através de mouse ou funções.
 *  - reguas
 *  - crosses
 *  - comportamento mestre-escravo
 *  - drop target para imagens
 *  - suporta seleções retângulares.
 *
 *
 *
 * Pode desenhar:
 *  - imagem de background
 *
 *
 *
 */
public class BaseCanvas extends JPanel implements ComponentListener, MouseInputListener, DropTargetListener
{
	private static Log log = LogFactory.getLog(BaseCanvas.class.getName());
	private static Log logCanvas = LogFactory.getLog("canvas");

	private Dimension dimImg = null;
	private Point ptLocation = null;
	private float zoomX = 1;
	private float zoomY = 1;

	private AffineTransform trafo = null;
	private AffineTransform invTrafo = null;
	private AffineTransform trafoIdent = null;

	private boolean bImgManListsEnabled = false;
	private boolean bImgManDrawerListsEnabled = false;
	private boolean bZoomEnabled = true;
	private boolean bDrawSelections = false;
	private boolean bUseRules = false;
	private boolean bUseCross = false;


	private BaseCanvas imMaster = null;
	private BufferedImage img = null;

	private float zoomRateX = (float) 1.2;
	private float zoomRateY = (float) 1.2;

	private Point ptIniDrag = null;
	private Point ptLast = null;
	private Point pLastDragged = null;

	private boolean bBeingDragged = false;

	private EventListenerList listenerList = null;
	private ArrayList listTopComponents = null;

	private ObjectHolder holderAffTrns = null;
	private JRuleHeader xRule = null;
	private JRuleHeader yRule = null;
	private DynamicCrossRenderer crossRenderer = null;

	// Não sei bem o que faz ...
	private ViewportSizeListener viewportListener = null;

	private TexturePaint textureFill = null;
	private BasicStroke strokeSelection = null;

	private boolean bPainted = false; 

	public BaseCanvas()
	{
		bPainted = false;
		dimImg = new Dimension(200, 200);
		ptLocation = new Point(0, 0);

		trafo = new AffineTransform();
		trafoIdent = new AffineTransform();
		try {invTrafo = trafo.createInverse();}
		catch (NoninvertibleTransformException ex) {invTrafo = trafo;}

		DropTarget dropTrgt = new DropTarget(this, DnDConstants.ACTION_COPY_OR_MOVE, this);
		this.bZoomEnabled = true;
		this.addComponentListener(this);
		this.addMouseListener(this);
		this.addMouseMotionListener(this);
		listenerList = new EventListenerList();
		listTopComponents = new ArrayList();

		holderAffTrns = new ObjectHolder(trafo, true);

		Dimension dimIni = new Dimension(200, 200);
		xRule = new JRuleHeader(JRuleHeader.HORIZONTAL_RULE, dimIni.width, holderAffTrns);
		yRule = new JRuleHeader(JRuleHeader.VERTICAL_RULE, dimIni.height, holderAffTrns);

		viewportListener = new ViewportSizeListener();
		crossRenderer = new DynamicCrossRenderer(this);
		crossRenderer.setTransformSource(holderAffTrns);

		// mistério ...
		BufferedImage bufImg = ImageUtil.convert2AlphaBufferedImage(ImageUtil.loadImageAsResource("/br/com/r4j/gui/textureFill.jpg"));
//		log.debug("bufImg = " + bufImg);
		Graphics2D gd2 = bufImg.createGraphics();
//		log.debug("gd2 = " + gd2);
		gd2.setColor(Color.green);
		gd2.fillRect(0, 0, 4, 4);
		WritableRaster alhpaRaster = bufImg.getAlphaRaster();
//		log.debug("alhpaRaster = " + alhpaRaster);
		for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++)
			alhpaRaster.setSample(i, j, 0, 250);
		textureFill = new TexturePaint(bufImg, new Rectangle(0, 0, 4, 4));
		strokeSelection = new BasicStroke(2);

		this.showRules(false);
		this.showCross(false);
		this.enableZoom(false);
		this.drawSelections(false);
		this.enableCanvasMouseListeners(false);
		this.enableRendererListeners(false);
	}


	public boolean isShowingRules()	{return bUseRules;}
	public void showRules(boolean bEn)
	{
		bUseRules = bEn;
		JScrollPane scrollPane = this.getScrollPane();
		if (scrollPane == null)
			return;
		if (bEn)
		{
			scrollPane.setColumnHeaderView(xRule);
			scrollPane.setRowHeaderView(yRule);
		}
		else
		{
			scrollPane.setColumnHeaderView(null);
			scrollPane.setRowHeaderView(null);
		}
		this.update();
	}


	public boolean isShowingCross()	{return bUseCross;}
	public void showCross(boolean bEn)
	{
		bUseCross = bEn;
		crossRenderer.setCrossEnabled(bEn);
		if (bUseCross)
		{
			if (imMaster == null)
				this.addMouseMotionListener(crossRenderer);
			else
				imMaster.getContentPane().addMouseMotionListener(crossRenderer);
		}
		else
		{
			if (imMaster == null)
				this.removeMouseMotionListener(crossRenderer);
			else
				imMaster.getContentPane().removeMouseMotionListener(crossRenderer);
		}

		this.update();
	}


	public void enableZoom(boolean bEn)	{bZoomEnabled = bEn;}
	public void drawSelections(boolean bEn)	{bDrawSelections = bEn;}
	public void enableCanvasMouseListeners(boolean bEn)	{bImgManListsEnabled = bEn;}
	public void enableRendererListeners(boolean bEn)	{bImgManDrawerListsEnabled = bEn;}


	public void enslave(BaseCanvas imMaster)
	{
		this.imMaster = imMaster;
		this.removeMouseListener(this);
		this.removeMouseMotionListener(this);
		this.removeMouseMotionListener(crossRenderer);
		imMaster.getContentPane().addMouseListener(this);
		imMaster.getContentPane().addMouseMotionListener(this);
		imMaster.getContentPane().addMouseMotionListener(crossRenderer);
	}


	public void setFree()
	{
		imMaster.getContentPane().removeMouseListener(this);
		imMaster.getContentPane().removeMouseMotionListener(this);
		imMaster.getContentPane().removeMouseMotionListener(crossRenderer);
		this.addMouseListener(this);
		this.addMouseMotionListener(this);
		this.addMouseMotionListener(crossRenderer);
		imMaster = null;
	}


	public BufferedImage getBackImage()	{return this.img;}
	public void setBackImage(BufferedImage img)
	{
		SampleModel smplModel = img.getSampleModel();
		if (!(smplModel instanceof SinglePixelPackedSampleModel))
			this.img = ImageUtil.convert2PackedBufferedImage(img, BufferedImage.TYPE_INT_RGB);
		else
			this.img = img;
		dimImg = new Dimension(img.getWidth(), img.getHeight());
		this.update(this.getGraphics());
	}

	
	public JComponent getContentPane()	{return this;}


/////////////////////////////////////////////////////////////////////////////////
// Métodos de contorle do zoom 

	public void zoom(float xZoom, float yZoom)
	{
		zoomX *= xZoom;
		zoomY *= yZoom;
		if (ptLocation != null)
		{
			ptLocation.x = (int) (ptLocation.x*xZoom);
			ptLocation.y = (int) (ptLocation.y*yZoom);
		}

		if (ptLocation != null && dimImg != null)
			this.setPreferredSize(new Dimension((int) (ptLocation.x + dimImg.width*zoomX), (int) (ptLocation.y + dimImg.height*zoomY)));

		trafo.setToScale(zoomX, zoomY);
		trafo.translate(ptLocation.x/zoomX, ptLocation.y/zoomY);
		try {invTrafo = trafo.createInverse();}
		catch (NoninvertibleTransformException ex) {invTrafo = trafo;}
		holderAffTrns.set(trafo);
/*
		if (this.getParent() != null)
			this.getParent().update(this.getParent().getGraphics());
		else
			this.update(this.getGraphics());
//*/			
		this.update();
	}


	public void moveImage(int dx, int dy)
	{
		if (ptLocation != null)
			ptLocation.translate(dx, dy);
		if (ptLocation!= null && dimImg != null)
			this.setPreferredSize(new Dimension((int) (ptLocation.x + dimImg.width*zoomX), (int) (ptLocation.y + dimImg.height*zoomY)));

		trafo.setToScale(zoomX, zoomY);
		trafo.translate(ptLocation.x/zoomX, ptLocation.y/zoomY);
		try {invTrafo = trafo.createInverse();}
		catch (NoninvertibleTransformException ex) {invTrafo = trafo;}
		holderAffTrns.set(trafo);

/*
		if (this.getParent() != null)
			this.getParent().update(this.getParent().getGraphics());
		else
			this.update(this.getGraphics());
//*/
		this.update();
	}


/////////////////////////////////////////////////////////////////////////////////
// Métodos sobreescritos e de renderização do Component


	public void erase()
	{
		int uniqueIdPaint = countPaint++;
		logCanvas.debug("erase "+uniqueIdPaint+">: " + bPainted);
//		log.debug(TraceUtil.currentStackTrace("\r\n"));
		Graphics2D g2dtrue = this.getCanvasGraphics(true);
		Graphics2D g2dfalse = this.getCanvasGraphics(false);
		if (g2dtrue != null && g2dfalse != null && !bPainted)
			this.fireErase(new RendererEvent(this, g2dtrue, g2dfalse, trafo, invTrafo));
		logCanvas.debug("erase "+uniqueIdPaint+">");
	}
	

	public void render()
	{
		int uniqueIdPaint = countPaint++;
		logCanvas.debug("render "+uniqueIdPaint+"> ... " + bPainted);
//		log.debug(TraceUtil.currentStackTrace("\r\n"));
		Graphics2D g2dtrue = this.getCanvasGraphics(true);
		Graphics2D g2dfalse = this.getCanvasGraphics(false);
		if (g2dtrue != null && g2dfalse != null && !bPainted)
			this.fireRender(new RendererEvent(this, g2dtrue, g2dfalse, trafo, invTrafo));
		else
			this.repaint();
		bPainted = false;
		logCanvas.debug("render "+uniqueIdPaint+"");
	}


	/**
	 * Permite escrever no background. as mudanças ficam pra sempre ...
	 */
	public Graphics2D getBackImageGraphics()
	{
		if (img != null)
			return img.createGraphics();
		else
			return null;
	}


	/**
	 * Permite escrever no canvas. Volátil
	 */
	public Graphics2D getCanvasGraphics(boolean bTransformForImage)
	{
		Graphics2D g2d = (Graphics2D) this.getGraphics();
		if (g2d != null && bTransformForImage)
		{
			Graphics2D g2dd = ((Graphics2D) g2d.create());
			g2dd.transform(trafo);
			return g2dd;
		}
		else
			return g2d;
	}

	private static int countPaint = 0;
	private Boolean bLock = new Boolean(true);
	private boolean bPainting = true;
	public void paint(Graphics g)
	{
		int uniqueIdPaint = countPaint++;
		logCanvas.debug("paint "+uniqueIdPaint+">");
/*		
		if (bPainting)
		{
			log.debug("already painting ...");
			return;
		}
//*/		
//		synchronized (bLock)
		{
			bPainting = true;
			bPainted = true;
			Graphics2D g2d = (Graphics2D) g;
			if (this.getViewport() != null && g2d != null)
			{
	//			log.debug("paint ...  -> " + g.getClipBounds());
	//			log.debug("this.getViewport().getSize()  -> " + this.getViewport().getSize());
	//			log.debug(TraceUtil.currentStackTrace("\r\n"));
				Dimension dimScreen = this.getViewport().getSize();
				g2d.setClip(0, 0, dimScreen.width, dimScreen.height);
				g2d.clearRect(0, 0, dimScreen.width, dimScreen.height);
				Graphics2D g2dUntrafo = (Graphics2D) g2d.create(); 
				g2d.transform(trafo);
	
				if (img != null && ptLocation != null)
				{
					if (bImgManDrawerListsEnabled)
					{
						RendererEvent e = new RendererEvent(this, g2d, g2dUntrafo, trafo, invTrafo);
						this.fireImageUpdatePerformed(e);
					}
					g2d.drawImage(img, trafoIdent, null);
				}
				crossRenderer.resetCross();
	
				if (bImgManDrawerListsEnabled)
				{
					RendererEvent e = new RendererEvent(this, g2d, g2dUntrafo, trafo, invTrafo);
					this.fireUpdatePerformed(e);
				}
	
				g2d.setTransform(trafoIdent);
				if (bDrawSelections && pLastDragged != null)
				{
					// alera o XOR mode ...
					this.drawSelection(g2d, this.getRect(ptIniDrag, pLastDragged));
				}
	//			log.debug("paint ends");
				bPainting = false;
			}
		}
		logCanvas.debug("paint "+uniqueIdPaint+">");
	}


	protected void update()
	{
//		log.debug("update ...");
//		log.debug(TraceUtil.currentStackTrace("\r\n"));
		synchronized (bLock)
		{
			if (this.getParent() != null && this.getParent().getGraphics() != null)
				this.getParent().update(this.getParent().getGraphics());
			else if (this.getGraphics() != null)
				this.update(this.getGraphics());
				
			Iterator itTopComponents = listTopComponents.iterator();
			while (itTopComponents.hasNext())
			{
				Component comp = (Component) itTopComponents.next();
				comp.repaint();
			}
		}
//		log.debug("update ends");
	}


	public void setPreferredSize(Dimension dim)
	{
		super.setPreferredSize(dim);
		xRule.setRuleLength(dim.width);
		yRule.setRuleLength(dim.height);
	}

	
	public void addNotify()
	{
        super.addNotify();
        this.configureEnclosingScrollPane();
    }


/////////////////////////////////////////////////////////////////////////////////
// Métodos internos

	protected JViewport getViewport()
	{
        final Container p = this.getParent();
        if (p instanceof JViewport)
		{
            final Container gp = p.getParent();
            if (gp instanceof JScrollPane)
			{
                JScrollPane scrollPane = (JScrollPane) gp;
                // Make certain we are the viewPort's view and not, for example, the rowHeaderView of the scrollPane - an implementor of fixed columns might do this.
                JViewport viewport = scrollPane.getViewport();
                if (viewport == null || viewport.getView() != this)
                    return null;
				else 
					return viewport;
            }
        }
		return null;
	}


	protected JScrollPane getScrollPane()
	{
        final Container p = this.getParent();
        if (p instanceof JViewport)
		{
            final Container gp = p.getParent();
            if (gp instanceof JScrollPane)
			{
                JScrollPane scrollPane = (JScrollPane) gp;
                // Make certain we are the viewPort's view and not, for example, the rowHeaderView of the scrollPane - an implementor of fixed columns might do this.
                JViewport viewport = scrollPane.getViewport();
                if (viewport == null || viewport.getView() != this)
                    return null;
				else 
					return scrollPane;
            }
        }
		return null;
	}

	
	protected void configureEnclosingScrollPane()
	{
		JScrollPane scrollPane = this.getScrollPane();
		if (scrollPane == null)
			return;

		if (bUseRules)
		{
			scrollPane.setColumnHeaderView(xRule);
			scrollPane.setRowHeaderView(yRule);
		}

		JViewport viewPort = this.getViewport();
		Dimension dimViewPort = viewPort.getExtentSize();

		viewPort.addChangeListener(viewportListener);
	}


	protected class ViewportSizeListener implements ChangeListener
	{
		public ViewportSizeListener()
		{
		}


		public void stateChanged(ChangeEvent e)
		{
			JViewport viewPort = (JViewport) e.getSource();
			Dimension dimViewPort = viewPort.getExtentSize();
		}
	}


	/** 
	 * precisa ir para utils ...
	 */
	private Rectangle getRect(Point ptIniDrag, Point pLastDragged)
	{
		Rectangle rectTmp = new Rectangle(0, 0, 0, 0);
		if (ptIniDrag.x > pLastDragged.x)
		{
			rectTmp.x = pLastDragged.x;
			rectTmp.width = ptIniDrag.x - pLastDragged.x;
		}
		else
		{
			rectTmp.x = ptIniDrag.x;
			rectTmp.width = pLastDragged.x - ptIniDrag.x;
		}
		if (ptIniDrag.y > pLastDragged.y)
		{
			rectTmp.y = pLastDragged.y;
			rectTmp.height = ptIniDrag.y - pLastDragged.y;
		}
		else
		{
			rectTmp.y = ptIniDrag.y;
			rectTmp.height = pLastDragged.y - ptIniDrag.y;
		}

		return rectTmp;
	}


	private void drawSelection(Graphics2D g2d, Rectangle rectDraw)
	{
		g2d.setXORMode(Color.white);
		g2d.setColor(Color.black);
		g2d.setPaint(textureFill);
		g2d.setStroke(strokeSelection);

		if (rectDraw.width > 2 && rectDraw.height > 2)
			g2d.fillRect(rectDraw.x + 1, rectDraw.y + 1, rectDraw.width - 1, rectDraw.height - 1);
		g2d.drawRect(rectDraw.x, rectDraw.y, rectDraw.width, rectDraw.height);
	}


/////////////////////////////////////////////////////////////////////////////////
// Controle de Listeners associados

	public void addRendererListener(RendererListener listener)
		{listenerList.add(RendererListener.class, listener);}
	public void removeRendererListener(RendererListener listener)	
		{listenerList.remove(RendererListener.class, listener);}
		
	/** 
	 * Adiciona componentes que aparecem por cima deste, então manda renderizar tudo quando move as coisas por aí.
	 *
	 * @param listener
	 */
	public void addTopComponent(Component compTop)
		{listTopComponents.add(compTop);}
	public void removeTopComponent(Component compTop)
		{listTopComponents.remove(compTop);}


	protected void fireImageUpdatePerformed(RendererEvent e)
	{
		if (bImgManDrawerListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == RendererListener.class)
					((RendererListener) listeners[i+1]).imageUpdatePerformed(e);
		}
	}


	protected void fireUpdatePerformed(RendererEvent e)
	{
		if (bImgManDrawerListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == RendererListener.class)
					((RendererListener) listeners[i+1]).updatePerformed(e);
		}
	}


	protected void fireRender(RendererEvent e)
	{
		if (bImgManDrawerListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == RendererListener.class)
					((RendererListener) listeners[i+1]).render(e);
		}
	}


	protected void fireErase(RendererEvent e)
	{
		if (bImgManDrawerListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == RendererListener.class)
					((RendererListener) listeners[i+1]).erase(e);
		}
	}


	public void addCanvasMouseListener(CanvasMouseListener listener)
		{listenerList.add(CanvasMouseListener.class, listener);}
	public void removeCanvasMouseListener(CanvasMouseListener listener)	
		{listenerList.remove(CanvasMouseListener.class, listener);}

	protected void fireMouseClicked(CanvasMouseEvent e)
	{
		if (bImgManListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == CanvasMouseListener.class)
					((CanvasMouseListener) listeners[i+1]).mouseClicked(e);
		}
	}
	protected void fireMousePressed(CanvasMouseEvent e)
	{
		if (bImgManListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == CanvasMouseListener.class)
					((CanvasMouseListener) listeners[i+1]).mousePressed(e);
		}
	}
	protected void fireMouseReleased(CanvasMouseEvent e)
	{
		if (bImgManListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == CanvasMouseListener.class)
					((CanvasMouseListener) listeners[i+1]).mouseReleased(e);
		}
	}
	protected void fireMouseMoved(CanvasMouseEvent e)
	{
		if (bImgManListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == CanvasMouseListener.class)
					((CanvasMouseListener) listeners[i+1]).mouseMoved(e);
		}
	}
	protected void fireMouseDragged(CanvasMouseEvent e)
	{
		if (bImgManListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == CanvasMouseListener.class)
					((CanvasMouseListener) listeners[i+1]).mouseDragged(e);
		}
	}
	protected void fireSelectionPerformed(CanvasSelectionEvent e)
	{
		if (bImgManListsEnabled)
		{
			Object [] listeners = listenerList.getListenerList();
			for (int i = listeners.length - 2; i >= 0; i -= 2)
				if (listeners[i] == CanvasMouseListener.class)
					((CanvasMouseListener) listeners[i+1]).selectionPerformed(e);
		}
	}





//////////////////////////////////////////////////////////////////////////
// Implementação dos listeneres implementados

	public void componentHidden(ComponentEvent e)
	{
	}


	public void componentMoved(ComponentEvent e)
	{
	}


	public void componentResized(ComponentEvent e)
	{
	}


	public void componentShown(ComponentEvent e)
	{
//		log.debug("componentShown");
		if (ptLocation == null)
			ptLocation = this.getLocation();
	}


	public void mouseDragged(MouseEvent e)
	{
		bBeingDragged = true;
//		log.debug("e.getButton() = " + e.getButton() + ", MouseEvent.BUTTON1 = " + MouseEvent.BUTTON1);
		if (bDrawSelections && ptIniDrag != null)
		{
			Graphics2D g2d = (Graphics2D) this.getGraphics();
			if (g2d != null)
			{
				if (pLastDragged != null)
					this.drawSelection(g2d, this.getRect(ptIniDrag, pLastDragged));

				pLastDragged = e.getPoint();
				this.drawSelection(g2d, this.getRect(ptIniDrag, pLastDragged));
//				log.debug("pLastDragged: " + pLastDragged + ", ptIniDrag: " + ptIniDrag);
			}
		}
		if (bImgManListsEnabled)
		{
			CanvasMouseEvent evtNew = null;
			try
				{evtNew = new CanvasMouseEvent(this, trafo.inverseTransform(e.getPoint(), null), e.getButton());}
			catch (NoninvertibleTransformException ex)
				{evtNew = new CanvasMouseEvent(this, e.getPoint(), e.getButton());}
			this.fireMouseDragged(evtNew);
		}
	}


	public void mouseMoved(MouseEvent e)
	{
		if (bImgManListsEnabled)
		{
			CanvasMouseEvent evtNew = null;
			try
				{evtNew = new CanvasMouseEvent(this, trafo.inverseTransform(e.getPoint(), null), e.getButton());}
			catch (NoninvertibleTransformException ex)
				{evtNew = new CanvasMouseEvent(this, e.getPoint(), e.getButton());}
			this.fireMouseMoved(evtNew);
		}
	}


	public void mouseClicked(MouseEvent e)
	{
		if (e.getButton() == MouseEvent.BUTTON1)
		{
			if (bBeingDragged)
			{
				bBeingDragged = false;
				if (bZoomEnabled)
					this.moveImage(ptLast.x - ptIniDrag.x, ptLast.y - ptIniDrag.y);
			}
			else
			{
				if (bZoomEnabled)
					this.zoom(zoomRateX, zoomRateY);
			}
		}
		else
		{
			if (bZoomEnabled)
				this.zoom(1/zoomRateX, 1/zoomRateY);
		}
		if (bImgManListsEnabled)
		{
			CanvasMouseEvent evtNew = null;
			try
				{evtNew = new CanvasMouseEvent(this, trafo.inverseTransform(e.getPoint(), null), e.getButton());}
			catch (NoninvertibleTransformException ex)
				{evtNew = new CanvasMouseEvent(this, e.getPoint(), e.getButton());}
			this.fireMouseClicked(evtNew);
		}
	}


	public void mouseEntered(MouseEvent e)
	{
	}


	public void mouseExited(MouseEvent e)
	{
	}


	public void mousePressed(MouseEvent e)
	{
		if (bDrawSelections && pLastDragged != null)
		{
			Graphics2D g2d = (Graphics2D) this.getGraphics();
			if (g2d != null)
			{
				this.drawSelection(g2d, this.getRect(ptIniDrag, pLastDragged));
			}
		}
		pLastDragged = null;
//		log.debug("e.getButton() = " + e.getButton() + ", MouseEvent.BUTTON1 = " + MouseEvent.BUTTON1);
		if (e.getButton() == MouseEvent.BUTTON1)
			ptIniDrag = e.getPoint();
		else
			ptIniDrag = null;

		if (bImgManListsEnabled)
		{
			CanvasMouseEvent evtNew = null;
			try
				{evtNew = new CanvasMouseEvent(this, trafo.inverseTransform(e.getPoint(), null), e.getButton());}
			catch (NoninvertibleTransformException ex)
				{evtNew = new CanvasMouseEvent(this, e.getPoint(), e.getButton());}
			this.fireMousePressed(evtNew);
		}
	}


	public void mouseReleased(MouseEvent e)
	{
//		log.debug("mouseReleased: e.getPoint() = " + e.getPoint());
		if (e.getButton() == MouseEvent.BUTTON1)
		{
			ptLast = e.getPoint();
			if (bBeingDragged)
			{
				if (bZoomEnabled)
				{
					this.moveImage(ptLast.x - ptIniDrag.x, ptLast.y - ptIniDrag.y);
				}
			}
		}
		if (bImgManListsEnabled)
		{
			if (ptLast != null && ptIniDrag != null)
			{
				CanvasSelectionEvent evtNew = null;
				try
					{evtNew = new CanvasSelectionEvent(this, trafo.inverseTransform(ptIniDrag, null), trafo.inverseTransform(ptLast, null), e.getButton());}
				catch (NoninvertibleTransformException ex)
					{evtNew = new CanvasSelectionEvent(this, ptIniDrag, ptLast, e.getButton());}
				this.fireSelectionPerformed(evtNew);
			}

			{
				CanvasMouseEvent evtNew = null;
				try
					{evtNew = new CanvasMouseEvent(this, trafo.inverseTransform(e.getPoint(), null), e.getButton());}
				catch (NoninvertibleTransformException ex)
					{evtNew = new CanvasMouseEvent(this, e.getPoint(), e.getButton());}
				this.fireMouseReleased(evtNew);
			}
		}
		bBeingDragged = false;
	}


	public void drop(DropTargetDropEvent e)
	{
//		log.debug("drop:begin:e = " + e);
		try
		{
//			e.acceptDrop(e.getDropAction());
			Transferable tr = e.getTransferable();
			DataFlavor df = tr.getTransferDataFlavors()[0];

			if (e.isDataFlavorSupported(DataFlavor.imageFlavor))
			{
				BufferedImage imgTmp = (BufferedImage) tr.getTransferData(DataFlavor.imageFlavor);
				e.acceptDrop(DnDConstants.ACTION_COPY_OR_MOVE);
				this.setBackImage(imgTmp);
				e.dropComplete(true);
			}
			else if (e.isDataFlavorSupported(DataFlavor.javaFileListFlavor))
			{
				e.acceptDrop(e.getDropAction());
				List listImgs = (List) tr.getTransferData(DataFlavor.javaFileListFlavor);
				if (listImgs.size() == 0)
				{
					JOptionPane.showMessageDialog(null, "Nenhum arquivo passado!");
				}
				else if (listImgs.size() > 1)
				{
					JOptionPane.showMessageDialog(null, "Suporta apenas uma imagem de cada vez!");
				}
				else
				{
					File fl = (File) listImgs.get(0);
					try
					{
						Image imgLoad = ImageUtil.loadImage(fl.getCanonicalPath());
						BufferedImage buffImg = ImageUtil.convert2BufferedImage(imgLoad, BufferedImage.TYPE_INT_RGB);
						this.setBackImage(buffImg);
						e.dropComplete(true);
					}
					catch (Exception xe)
					{
						xe.printStackTrace();
						JOptionPane.showMessageDialog(null, "Não é um formato suportado!");
					}
				}
			}
			else
			{
				e.rejectDrop();
			}
		}
		catch(IOException ioe)
		{
			log.error("drop:error", ioe);
		}
		catch(UnsupportedFlavorException ufe)
		{
			log.error("drop:error", ufe);
		}
	}


    public void dragEnter(DropTargetDragEvent e) { }

    public void dragExit(DropTargetEvent e) { }

    public void dragOver(DropTargetDragEvent e) { }

    public void dropActionChanged(DropTargetDragEvent e) { }

}

 
