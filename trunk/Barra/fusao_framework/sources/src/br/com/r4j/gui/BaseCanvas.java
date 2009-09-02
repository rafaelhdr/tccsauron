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
 * @modelguid {7332D8EA-3A3C-4D9F-AD4D-F2A0F20EACD1}
 */
public class BaseCanvas extends JPanel implements ComponentListener, MouseInputListener, DropTargetListener
{
	/** @modelguid {44CDE8FB-CB09-4F84-BF5D-99A017772B8B} */
	private static Log log = LogFactory.getLog(BaseCanvas.class.getName());
	/** @modelguid {42A6945D-2B29-43BB-8052-6F237692347D} */
	private static Log logCanvas = LogFactory.getLog("canvas");

	/** @modelguid {B0C189EF-1350-472E-A3EA-8D1F5756FADE} */
	private Dimension dimImg = null;
	/** @modelguid {F252229C-90F1-4A8C-80FA-6FB82DCBAAB7} */
	private Point ptLocation = null;
	/** @modelguid {3797EC51-EB16-49C5-82B9-40C688B9EA0B} */
	private float zoomX = 1;
	/** @modelguid {678E4185-9E8E-426E-A885-975D0A227FF3} */
	private float zoomY = 1;

	/** @modelguid {95DD8BEC-7FF8-4DFD-9524-A6E51E4C6F56} */
	private AffineTransform trafo = null;
	/** @modelguid {D183E4C1-D7AC-4D7E-B8EB-CFCE23F894BD} */
	private AffineTransform invTrafo = null;
	/** @modelguid {0CDE7BBB-7EE1-485B-857D-04F2BB48CEFC} */
	private AffineTransform trafoIdent = null;

	/** @modelguid {A92D7E48-D261-4A5E-A13B-B69AF80A4B73} */
	private boolean bImgManListsEnabled = false;
	/** @modelguid {697696AC-D314-4D5C-9798-13BF43597C4A} */
	private boolean bImgManDrawerListsEnabled = false;
	/** @modelguid {296871EB-0D4B-41E9-A059-12F553B7FD59} */
	private boolean bZoomEnabled = true;
	/** @modelguid {C053D4B3-7EBD-4CD1-89A6-905F1340B8E2} */
	private boolean bDrawSelections = false;
	/** @modelguid {022444BA-AC53-4379-98D4-EBE8323A6CB0} */
	private boolean bUseRules = false;
	/** @modelguid {479D1A7F-9F57-4270-B4D7-802E7522CFF7} */
	private boolean bUseCross = false;


	/** @modelguid {035F3E2B-7219-47FC-9732-57D5F1328366} */
	private BaseCanvas imMaster = null;
	/** @modelguid {60D28876-18E6-4248-BBD1-8299D220E2AD} */
	private BufferedImage img = null;

	/** @modelguid {4C2577E0-FC0E-4D27-BF73-D44608C4EF8B} */
	private float zoomRateX = (float) 1.2;
	/** @modelguid {A80E7DE7-BC8B-4400-8092-CDB240BC9117} */
	private float zoomRateY = (float) 1.2;

	/** @modelguid {CB8A60BC-1DDD-4955-9650-7D6AD17B791C} */
	private Point ptIniDrag = null;
	/** @modelguid {23D00ED5-94B2-4612-9107-EE95DC6B3EBF} */
	private Point ptLast = null;
	/** @modelguid {C15A74CE-E10E-4628-9466-6BAE33C91A8F} */
	private Point pLastDragged = null;

	/** @modelguid {005AD240-5433-4273-92EF-068CF59B9E75} */
	private boolean bBeingDragged = false;

	/** @modelguid {526D8EE9-760B-45D6-ACE9-FF71AE509B88} */
	private EventListenerList listenerList = null;
	/** @modelguid {04270E40-27EB-4620-9A62-5B9638DBF13B} */
	private ArrayList listTopComponents = null;

	/** @modelguid {3ED7E30F-CC70-4C27-ADB2-5178AAF291B9} */
	private ObjectHolder holderAffTrns = null;
	/** @modelguid {0D61A7CB-F053-4D07-883B-DB0C1B4F3D03} */
	private JRuleHeader xRule = null;
	/** @modelguid {B3FB9996-8DE2-4DE8-894E-F49DEAEEF05D} */
	private JRuleHeader yRule = null;
	/** @modelguid {DCB200E4-11E9-40DB-9BA0-4DFD56BD363C} */
	private DynamicCrossRenderer crossRenderer = null;

	// Não sei bem o que faz ...
	/** @modelguid {3DC3906E-A572-492D-8CC7-E57F609DD86C} */
	private ViewportSizeListener viewportListener = null;

	/** @modelguid {28F55303-4F53-4971-9D00-B0FA9750CD42} */
	private TexturePaint textureFill = null;
	/** @modelguid {DD92374A-69D6-4FEE-A996-2923DFF8FA92} */
	private BasicStroke strokeSelection = null;

	/** @modelguid {CC0F0127-AC11-446D-A2B8-3508E3C2CC7A} */
	private boolean bPainted = false; 

	/** @modelguid {1F1DF7BA-3546-4995-BB8D-5758B0B62FD2} */
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


	/** @modelguid {1567B3D9-8DCE-49D6-858E-C042287597E6} */
	public boolean isShowingRules()	{return bUseRules;}
	/** @modelguid {8D3B9854-3480-4902-A00F-17AAA460BFF4} */
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


	/** @modelguid {A0E3860A-0CE1-4A52-A173-44B26688A111} */
	public boolean isShowingCross()	{return bUseCross;}
	/** @modelguid {CAC638C4-DE0A-4EB4-ADAC-7F8C7AD6EF1B} */
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


	/** @modelguid {125BE2B3-2DC9-4C1C-B018-32ED1389CB9A} */
	public void enableZoom(boolean bEn)	{bZoomEnabled = bEn;}
	/** @modelguid {3E182130-EFE9-4000-ADA6-917EC5F899AB} */
	public void drawSelections(boolean bEn)	{bDrawSelections = bEn;}
	/** @modelguid {1EA6C179-C863-4558-B206-E912EBD8D58C} */
	public void enableCanvasMouseListeners(boolean bEn)	{bImgManListsEnabled = bEn;}
	/** @modelguid {3FA0F7C3-CE8D-418D-B5BD-DBC4FE6D8B97} */
	public void enableRendererListeners(boolean bEn)	{bImgManDrawerListsEnabled = bEn;}


	/** @modelguid {CBC42CBA-D61B-4CAB-A2E1-99836917625C} */
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


	/** @modelguid {03D16D04-7A38-4CBA-8467-DEB1950147D8} */
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


	/** @modelguid {9CF78861-00C7-41D4-8C2C-AAA7DE13AE1C} */
	public BufferedImage getBackImage()	{return this.img;}
	/** @modelguid {D7DF5F77-5499-41AD-9D6F-E344C48CABFF} */
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

	
	/** @modelguid {46FCD9D2-5452-4D1F-BD00-32D322EC4F77} */
	public JComponent getContentPane()	{return this;}


/////////////////////////////////////////////////////////////////////////////////
// Métodos de contorle do zoom 

	/** @modelguid {B25C0FCA-07E0-47CC-A326-AD2EDEDB6321} */
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


	/** @modelguid {0A6374CD-983A-4BB1-9AF0-055E55FAE71F} */
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


	/** @modelguid {C24D3641-53B5-4A81-B980-4C82A816729F} */
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
	

	/** @modelguid {1278C875-2A65-4FEF-AE84-6A8B4D9331CB} */
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
		{
			logCanvas.debug("repaint!!");
			this.repaint();
		}
		bPainted = false;
		logCanvas.debug("render "+uniqueIdPaint+"");
	}


	/**
	 * Permite escrever no background. as mudanças ficam pra sempre ...
	 * @modelguid {E9A71D28-18DC-4A62-9757-51366A15D756}
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
	 * @modelguid {51FD14A0-7857-4C96-A3F7-50730996A786}
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

	/** @modelguid {347B1B71-3D69-49C5-988D-4536D0E8A84A} */
	private static int countPaint = 0;
	/** @modelguid {26504B4A-0B31-4EB9-98EC-D6F56F604CC7} */
	private Boolean bLock = new Boolean(true);
	/** @modelguid {E02C0110-54F0-4BD0-B746-594CF9BC29A7} */
	private boolean bPainting = true;
	/** @modelguid {88989302-003C-49DA-B7C4-F10BD86E990B} */
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


	/** @modelguid {B47B4ED0-56A6-4F8A-8AA1-43AEC3014C02} */
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


	/** @modelguid {CC7F60CA-AB0B-4EB1-9BC9-68EDEA0EC58F} */
	public void setPreferredSize(Dimension dim)
	{
		super.setPreferredSize(dim);
		xRule.setRuleLength(dim.width);
		yRule.setRuleLength(dim.height);
	}

	
	/** @modelguid {97B83C04-7DA8-4B1E-88B6-7EABED6E39EF} */
	public void addNotify()
	{
        super.addNotify();
        this.configureEnclosingScrollPane();
    }


/////////////////////////////////////////////////////////////////////////////////
// Métodos internos

	/** @modelguid {0A107CE2-225D-4E5B-9E23-BA9D922D81B2} */
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


	/** @modelguid {0A19609E-FA27-4DA2-9CFF-AE191C98FFC0} */
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

	
	/** @modelguid {1538DAFD-B922-4B2A-8728-FF0FFA2527D1} */
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


	/** @modelguid {01CDB0F7-D239-414B-AACE-5EE2911372FC} */
	protected class ViewportSizeListener implements ChangeListener
	{
		/** @modelguid {1F7B1F9F-DAFB-4F68-8B2F-84E0CA6E0F5B} */
		public ViewportSizeListener()
		{
		}


		/** @modelguid {C59E8B56-D44A-4771-BDDA-4D4996E59678} */
		public void stateChanged(ChangeEvent e)
		{
			JViewport viewPort = (JViewport) e.getSource();
			Dimension dimViewPort = viewPort.getExtentSize();
		}
	}


	/** 
	 * precisa ir para utils ...
	 * @modelguid {C4279180-A6B4-4655-94A8-5962B610AB15}
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


	/** @modelguid {4224E677-23B3-4924-A53D-0BDF514E34D0} */
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

	/** @modelguid {9A5B5EF5-C99B-434F-A65E-F8394B69CEC2} */
	public void addRendererListener(RendererListener listener)
		{listenerList.add(RendererListener.class, listener);}
	/** @modelguid {332BAFAF-06C4-4FDA-BABD-B110457697B9} */
	public void removeRendererListener(RendererListener listener)	
		{listenerList.remove(RendererListener.class, listener);}
		
	/** 
	 * Adiciona componentes que aparecem por cima deste, então manda renderizar tudo quando move as coisas por aí.
	 *
	 * @param listener
	 * @modelguid {471EC664-448A-4830-8E33-0404C510840A}
	 */
	public void addTopComponent(Component compTop)
		{listTopComponents.add(compTop);}
	/** @modelguid {33E2CD34-436A-433A-BADB-4C5F3B7C74C4} */
	public void removeTopComponent(Component compTop)
		{listTopComponents.remove(compTop);}


	/** @modelguid {76180AEB-757B-453B-95E8-D2BBF5EC4899} */
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


	/** @modelguid {03A40882-F6D0-4AAE-AF15-BE1B498632F6} */
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


	/** @modelguid {5B923108-A529-4B1B-9F1C-56E7FF9D20F3} */
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


	/** @modelguid {51198E4E-47DC-49ED-BC69-C57C4BE4FD3F} */
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


	/** @modelguid {FE361E64-96C2-4F0A-8BE3-B3F84E96D009} */
	public void addCanvasMouseListener(CanvasMouseListener listener)
		{listenerList.add(CanvasMouseListener.class, listener);}
	/** @modelguid {7FBE972F-3917-4605-83B8-F15B7DBF24AF} */
	public void removeCanvasMouseListener(CanvasMouseListener listener)	
		{listenerList.remove(CanvasMouseListener.class, listener);}

	/** @modelguid {50FEBBE3-1434-4369-A7AE-8ED47CA19CEC} */
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
	/** @modelguid {E7CE7975-4C89-49BE-931F-CBBC16F45A93} */
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
	/** @modelguid {0C234B4A-B094-4144-930E-D09DEC875FD5} */
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
	/** @modelguid {81643027-8995-4B53-B813-7214FD9F23BB} */
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
	/** @modelguid {800AFE80-9E94-4242-A8C0-5043BBAC7302} */
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
	/** @modelguid {252975D1-F5D1-4E66-A848-D64FD14B2AB0} */
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

	/** @modelguid {298D9220-3FBB-4545-9CEB-9C15FFEF6375} */
	public void componentHidden(ComponentEvent e)
	{
	}


	/** @modelguid {F1AE9728-EF00-43BE-AFC2-3B5CA26C101D} */
	public void componentMoved(ComponentEvent e)
	{
	}


	/** @modelguid {7BCFB251-AB84-4182-8B7F-E2D3515D0515} */
	public void componentResized(ComponentEvent e)
	{
	}


	/** @modelguid {237BA801-9494-4A49-9AC3-A3628332B1BD} */
	public void componentShown(ComponentEvent e)
	{
//		log.debug("componentShown");
		if (ptLocation == null)
			ptLocation = this.getLocation();
	}


	/** @modelguid {A05A861E-4A7B-4D95-AF55-F565B7CD37EE} */
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


	/** @modelguid {E66EF844-A941-42BD-B329-DE7F76865ACA} */
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


	/** @modelguid {44D5475E-0007-4FBD-A520-816EE8F9E388} */
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


	/** @modelguid {DBC1E573-3AC2-4AAE-BCDE-E272E1B4E94E} */
	public void mouseEntered(MouseEvent e)
	{
	}


	/** @modelguid {1BAEBD41-C4E3-4C08-9EAA-FB256C3B13E0} */
	public void mouseExited(MouseEvent e)
	{
	}


	/** @modelguid {F6B244E6-32A2-4B55-BF6F-FCD47A13DDD6} */
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


	/** @modelguid {4FDC9750-479B-4B7C-B9A2-755F47FC6C5E} */
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
					if (ptLast != null && ptIniDrag != null)
						this.moveImage(ptLast.x - ptIniDrag.x, ptLast.y - ptIniDrag.y);
					else
					{
						log.error("BaseCanvas.mouseReleased: ptLast = " + ptLast + ", ptIniDrag = " + ptIniDrag);
					}
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


	/** @modelguid {55A7E3B0-6D9D-432E-AFED-AB0386C07135} */
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


	/** @modelguid {71EA361C-3CE2-4DA7-ADFF-AE9FCCCDAF86} */
    public void dragEnter(DropTargetDragEvent e) { }

	/** @modelguid {553BAF22-703C-40FA-895E-E99A18BAADE0} */
    public void dragExit(DropTargetEvent e) { }

	/** @modelguid {4DB763CE-D0FD-4133-AF72-DB36DC74DED4} */
    public void dragOver(DropTargetDragEvent e) { }

	/** @modelguid {CE5713F3-7A64-4426-AC78-07B9585D57E8} */
    public void dropActionChanged(DropTargetDragEvent e) { }

}

 
