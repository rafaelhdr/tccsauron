package br.com.r4j.robosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.EventListenerList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;

import br.com.r4j.math.*;
import br.com.r4j.commons.draw.ShapesUtil;
import br.com.r4j.commons.parser.TokenLineStreamParser;
import br.com.r4j.gui.RendererEvent;
import br.com.r4j.gui.RendererListener;
import br.com.r4j.math.FunctionsR;
import br.com.r4j.math.geom.CDLLPolygon;
import br.com.r4j.math.geom.GeomOperations;
import br.com.r4j.math.geom.GeomUtils;
import br.com.r4j.math.geom.IPolygon;
import br.com.r4j.math.geom.gui.RenderablePolygonInfo;
import br.com.r4j.research.vline.*;
import br.com.r4j.research.*;
import br.com.r4j.research.image.sequence.*;

import br.com.r4j.configurator.Configurator;
import br.com.r4j.configurator.ConfiguratorException;
import br.com.r4j.configurator.PropertiesHolder;


/**
 * 
 * @author giord
 *
 * O mapa é formado por um conjunto de paredes dispostas em qualquer posição, além de um conjunto 
 * de pontos que indicam retas visuais identificáveis no ambiente, além das quinas das paredes. 
 *
 * arquivo do mapa:
 *
 */
public class WorldMap extends TokenLineStreamParser implements RendererListener
{
	private static Log log = LogFactory.getLog(WorldMap.class.getName());
	private static Log logSens = LogFactory.getLog("sonar");
	private static Log logModel = LogFactory.getLog("sonarmodel");
	private static Log logVisionMarcoCount = LogFactory.getLog("marco_visao");

	private ArrayList listPolygons = new ArrayList();
	private ArrayList listSegs = new ArrayList();
	private ArrayList listPointsVLine = new ArrayList();

	private IPolygon polyOuter = null;

	private double [] arrayOrderedTheta = null;
	private Wall [] arrayThetaOrderedWall = null;

	private double [] arrayOrderedLineY = null;
	private VLineRef [] arrayOrderedLine = null;


	private EventListenerList listenerList = null;
	private File flMapFile = null;

	private String strNameMyName = null;

	private double wallRejectionValue = 1.645;
	private double wallRejectionValue2 = wallRejectionValue*wallRejectionValue;

	private int xIni = -1;
	private int yIni = -1;

	private int xIniEst = -1;
	private int yIniEst = -1;
	private double thetaIniEst = -1;


	public WorldMap()
	{
		listenerList = new EventListenerList();
		strNameMyName = "Mapa";
	}


	public void setMapFile(String strFile) throws IOException
	{
		this.setMapFile(new File(strFile));
	}

		
	public void setMapFile(File fFile) throws IOException
	{
		xIni = -1;
		yIni = -1;
		listPolygons.clear();
		listSegs.clear();
		listPointsVLine.clear();

		this.flMapFile = fFile;
		ArrayList listLists = new ArrayList();
		int width = -1, height = -1;
		xIni = -1; yIni = -1;

		char [] arrayDataBuffer = new char[200];
		String strWord = null;
//		if (log.isDebugEnabled()) log.debug("start parse:");
		try
		{
			FileReader fRead = new FileReader(flMapFile);

			strWord = this.getWord(fRead, arrayDataBuffer);
			while (strWord != null)
			{
//				if (log.isDebugEnabled()) log.debug("strWord:" + strWord);
				if (strWord.startsWith(";"))
				{
					this.jumpToNextLine(fRead, arrayDataBuffer);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("width"))
				{
					width = this.getInt(fRead, arrayDataBuffer);
//					if (log.isDebugEnabled()) log.debug("width = " + width);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("height"))
				{
					height = this.getInt(fRead, arrayDataBuffer);
//					if (log.isDebugEnabled()) log.debug("height = " + height);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("position"))
				{
					xIni = this.getInt(fRead, arrayDataBuffer);
					yIni = this.getInt(fRead, arrayDataBuffer);
					if (log.isDebugEnabled())
						log.debug("xIni = " + xIni + ", yIni = " + yIni + ", theta = " + strWord);
					strWord = this.getWord(fRead, arrayDataBuffer);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("estimate"))
				{
					xIniEst = this.getInt(fRead, arrayDataBuffer);
					yIniEst = this.getInt(fRead, arrayDataBuffer);
					thetaIniEst = this.getDouble(fRead, arrayDataBuffer);
					if (log.isDebugEnabled())
						log.debug("xIniEst = " + xIniEst + ", yIniEst = " + yIniEst + ", thetaIniEst = " + thetaIniEst);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("vline"))
				{
					int xPoint = this.getInt(fRead, arrayDataBuffer);
					int yPoint = this.getInt(fRead, arrayDataBuffer);
					int thNormalInv = this.getInt(fRead, arrayDataBuffer);
					log.debug("vline: xPoint:" + xPoint + ", yPoint:" + yPoint + ". thNormalInv: " + thNormalInv);
					int band = this.getInt(fRead, arrayDataBuffer);
					VLineRef lRef = new VLineRef(xPoint, yPoint, band, thNormalInv*1.0*Math.PI/180.0);
					listPointsVLine.add(lRef);
					char ind = this.getChar(fRead, arrayDataBuffer);
					while (ind != 'E')
					{
						int [] arrayPerfil = null;
						if (ind == 'R')
						{
							lRef.rLeft = this.getInt(fRead, arrayDataBuffer);
							lRef.rRight = this.getInt(fRead, arrayDataBuffer);
							log.debug("vline: R");
						}
						else if (ind == 'G')
						{
							lRef.gLeft = this.getInt(fRead, arrayDataBuffer);
							lRef.gRight = this.getInt(fRead, arrayDataBuffer);
							log.debug("vline: G");
						}
						else if (ind == 'B')
						{
							lRef.bLeft = this.getInt(fRead, arrayDataBuffer);
							lRef.bRight = this.getInt(fRead, arrayDataBuffer);
							log.debug("vline: B");
						}
						else
						{
							log.error("!!ind = " + ind);
						}
						ind = this.getChar(fRead, arrayDataBuffer);
//						if (log.isDebugEnabled()) log.debug("ind = " + ind);
					}
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else
				{
					ArrayList listPoints = new ArrayList();
					// ponto 1
					int x1 = this.getInt(strWord), y1 = this.getInt(fRead, arrayDataBuffer);
					listPoints.add(new Point(x1, y1));

					// ponto 2
					int x2 = this.getInt(fRead, arrayDataBuffer), y2 = this.getInt(fRead, arrayDataBuffer);
					if (this.nextNonWhiteSpaceIs(';', fRead))
						this.jumpToNextLine(fRead, arrayDataBuffer);
					while (this.countNextsEnters(fRead) == 1)
					{
						// ponto 1
						x1 = this.getInt(fRead, arrayDataBuffer); y1 = this.getInt(fRead, arrayDataBuffer);
						listPoints.add(new Point(x1, y1));

						// ponto 2
						x2 = this.getInt(fRead, arrayDataBuffer); y2 = this.getInt(fRead, arrayDataBuffer);
						if (this.nextNonWhiteSpaceIs(';', fRead))
							this.jumpToNextLine(fRead, arrayDataBuffer);
					}
					listLists.add(listPoints);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
			}
//			if (log.isDebugEnabled()) log.debug("end parse");

			TreeMap mapTheta2Wall = new TreeMap();

			List listPoints = new ArrayList();
			listPoints.add(new Point(0, 0));
			listPoints.add(new Point(width, 0));
			listPoints.add(new Point(width, height));
			listPoints.add(new Point(0, height));
			listLists.add(listPoints);

			ArrayList listSegsTmp = new ArrayList();
			Iterator itLists = listLists.iterator();
			while (itLists.hasNext())
			{
				listPoints = (List) itLists.next();
/*				
				Iterator itPoints = listPoints.iterator();
				while (itPoints.hasNext())
				{
					Point pt = (Point) itPoints.next();
					pt.translate(-xIni, -yIni);
				}
//*/				
				IPolygon polyTmp = new CDLLPolygon(listPoints);
				listPolygons.add(polyTmp);
				GeomUtils.addAllSegments(listSegsTmp, polyTmp);
				if (logSens.isDebugEnabled()) logSens.debug("polyTmp:" + polyTmp);
			}

			Iterator itSegsTmp = listSegsTmp.iterator();
			Line2D.Double lineAnt = (Line2D.Double) listSegsTmp.get(listSegsTmp.size() - 1);
			Wall wallPrevious = null;
			while (itSegsTmp.hasNext())
			{
				Line2D.Double line = (Line2D.Double) itSegsTmp.next();

				Wall wall = new Wall();
				wall.setX1(line.getX1());
				wall.setX2(line.getX2());
				wall.setY1(line.getY1());
				wall.setY2(line.getY2());
				wall.setIs1Corner(GeomOperations.isObtuse(lineAnt.getP1(), line.getP1(), line.getP2()));
				wall.calculate();

			
				// Coxa braba pra passar a busca para o range 0 - 180.
				double thetaTmp = wall.getTheta();
				if (thetaTmp < 0)
					thetaTmp += Math.PI*2;
				if (thetaTmp >= Math.PI)
					thetaTmp -= Math.PI;

				if (logSens.isDebugEnabled()) logSens.debug("wall: " + wall + ", thetaTmp: " + thetaTmp);
					 
				Double dblTheta = new Double(thetaTmp);
				ArrayList listWalls = (ArrayList) mapTheta2Wall.get(dblTheta);
				if (listWalls == null)
				{
					listWalls = new ArrayList(); 
					mapTheta2Wall.put(dblTheta, listWalls);
				}
				listWalls.add(wall);

				if (wallPrevious != null)
				{
					wallPrevious.setIs2Corner(GeomOperations.isObtuse(lineAnt.getP1(), lineAnt.getP2(), line.getP1()));
					wall.setPrevious(wallPrevious);
					wallPrevious.setNext(wall);
				}

				listSegs.add(wall);

				wallPrevious = wall;
				lineAnt = line;
			}
			Wall wall1 = (Wall) listSegs.get(0);
			Line2D.Double line1 = (Line2D.Double) listSegsTmp.get(0);
			wall1.setPrevious(wallPrevious);
			wall1.setIs2Corner(GeomOperations.isObtuse(lineAnt.getP1(), lineAnt.getP2(), line1.getP1()));

			polyOuter = (IPolygon) listPolygons.get(listPolygons.size() - 1);
//			if (log.isDebugEnabled()) log.debug("listPolygons.size() = " + listPolygons.size());

			TreeMap mapY2Line = new TreeMap();
			Iterator itVLines = listPointsVLine.iterator();
			while (itVLines.hasNext())
			{
				VLineRef ptVLine = (VLineRef) itVLines.next();
				ArrayList listLines = (ArrayList) mapY2Line.get(new Double(ptVLine.getY()));
				if (listLines == null)
				{
					listLines = new ArrayList(); 
					mapY2Line.put(new Double(ptVLine.getY()), listLines);
				}
				listLines.add(ptVLine);
				
				Iterator itWalls = listSegs.iterator();
				while (itWalls.hasNext())
				{
					Wall wally = (Wall) itWalls.next();
					if (GeomOperations.intersectCircle(wally, ptVLine.getX(), ptVLine.getY(), 30))
					{
						wally.addVerticalLine(ptVLine);
						break;
					}
				}
			}

			arrayOrderedTheta = new double[listSegs.size()];
			arrayThetaOrderedWall = new Wall[listSegs.size()];
			Iterator itThetas = mapTheta2Wall.keySet().iterator();
			int counterTheta = 0;
			while (itThetas.hasNext())
			{
				Double dblTheta = (Double) itThetas.next();
				ArrayList listWalls = (ArrayList) mapTheta2Wall.get(dblTheta);
				Iterator itWalls = listWalls.iterator();
				while  (itWalls.hasNext())
				{
					Wall wally = (Wall) itWalls.next();
					arrayThetaOrderedWall[counterTheta] = wally;
					arrayOrderedTheta[counterTheta] = dblTheta.doubleValue();
					if (logSens.isDebugEnabled())
					{
						logSens.debug("arrayThetaOrderedWall[" + counterTheta + "]: " + arrayThetaOrderedWall[counterTheta]);
						logSens.debug("arrayOrderedTheta[" + counterTheta + "]: " + arrayOrderedTheta[counterTheta]);
					}
					counterTheta++;
				}
			}

			arrayOrderedLineY = new double[listPointsVLine.size()];
			arrayOrderedLine = new VLineRef[listPointsVLine.size()];
			Iterator itYs = mapY2Line.keySet().iterator();
			int counterY = 0;
			while (itYs.hasNext())
			{
				Double dblY = (Double) itYs.next();
				ArrayList listLines = (ArrayList) mapY2Line.get(dblY);
				Iterator itLines = listLines.iterator();
				while  (itLines.hasNext())
				{
					VLineRef line = (VLineRef) itLines.next();
					arrayOrderedLine[counterY] = line;
					arrayOrderedLineY[counterY] = dblY.doubleValue();
					counterY++;
				}
				
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
			log.debug("erro", e);
			System.err.println("Não foi possível caregar o mapa!");
			return;
		}
	}


	public List getVLines()
	{
		return listPointsVLine;
	}


	public Rectangle2D getBoundingBox()
	{
		Rectangle2D rect = new Rectangle2D.Double(polyOuter.getX(0), polyOuter.getY(0), polyOuter.getX(2), polyOuter.getY(2));
		return rect;
	}


	public void addChangeListener(ChangeListener listener)
		{listenerList.add(ChangeListener.class, listener);}
	public void removeChangeListener(ChangeListener listener)
		{listenerList.remove(ChangeListener.class, listener);}


	protected void fireStateChanged(ChangeEvent e)
	{
		Object [] listeners = listenerList.getListenerList();
		for (int i = listeners.length - 2; i >= 0; i -= 2)
			if (listeners[i] == ChangeListener.class)
				((ChangeListener) listeners[i+1]).stateChanged(e);
	}


	public void imageUpdatePerformed(RendererEvent e)
	{
	}


	/**
	 * Chamado cada vez que todo o resto já foi desenhado (Paint mode).
	 */
	public void updatePerformed(RendererEvent e)
	{
		Graphics2D g2d = e.getGraphics();
		AffineTransform trafo = g2d.getTransform();

		BasicStroke strokeObjectOutline = new BasicStroke(1f);
//		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER);
//		Composite cOri = g2d.getComposite();
//		g2d.setComposite(ac);
		Iterator itRenderPolyInfo = listPolygons.iterator();
		while (itRenderPolyInfo.hasNext())
		{
			IPolygon poly = (IPolygon) itRenderPolyInfo.next();
			if (polyOuter == poly)
				continue;
			RenderablePolygonInfo renderPolyInfo = new RenderablePolygonInfo(true, false, Color.white, Color.black, strokeObjectOutline, poly);
			renderPolyInfo.draw(g2d);
		}
//		g2d.setComposite(cOri);

//		strokeObjectOutline = new BasicStroke(2f);
//		RenderablePolygonInfo renderPolyInfoOutline = new RenderablePolygonInfo(true, false, Color.white, Color.black, strokeObjectOutline, polyOuter);
//		renderPolyInfoOutline.draw(g2d);

		Shape shpCircle = ShapesUtil.createCircle(6);
//		shpCircle =  e.untransform(shpCircle);
		BasicStroke strokeVLines = new BasicStroke(1f);
		g2d.setStroke(strokeVLines);
		Iterator itPointsVLine = listPointsVLine.iterator();
		while (itPointsVLine.hasNext())
		{
			VLineRef vline = (VLineRef) itPointsVLine.next();
			g2d.setColor(Color.red);
//			AffineTransform trafoTmp = AffineTransform.getTranslateInstance(vline.getX(), vline.getY());
//			trafoTmp.concatenate(trafo); 
//			Shape shpTrans = trafoTmp.createTransformedShape(shpCircle);
			Shape shpTrans = e.translateAndMaintainShapeSize(vline.getX(), vline.getY(), shpCircle);
			g2d.fill(shpTrans);
			g2d.setColor(Color.blue);
			g2d.draw(shpTrans);
		}
	}


	/**
	 * Chamado cada vez que é necessário redesenhar o objeto (XOR mode).
	 */
	public void render(RendererEvent e)
	{
	}


	/**
	 * Chamado cada vez que é necessário apagar o objeto (XOR mode).
	 */
	public void erase(RendererEvent e)
	{
	}


	public String getName()
	{
		return strNameMyName;
	}
	

	public AbstractDoubleVector getInitialLocalization()
	{
		AbstractDoubleVector initialPosition = new DoubleVector(3);
		initialPosition.setComponent(0, xIni);
		initialPosition.setComponent(1, yIni);
		initialPosition.setComponent(2, 0);
		return initialPosition;
	}


	public AbstractDoubleVector getInitialLocalizationEstimate()
	{
		AbstractDoubleVector initialPosition = new DoubleVector(3);
		initialPosition.setComponent(0, xIniEst);
		initialPosition.setComponent(1, yIniEst);
		initialPosition.setComponent(2, thetaIniEst*Math.PI/180);
		return initialPosition;
	}


	public AbstractDoubleSquareMatrix getInitialLocalizationCovar()
	{
		double xStdDev = 120, yStdDev = 120, thetaStdDev = 1;
		Configurator conf = Configurator.getInstance();
		PropertiesHolder props = conf.getPropertiesHolder();
		if (props.containsProperty("/robosim/ini_pos_covar/x_std_dev_mm"))
			xStdDev = props.getDoubleProperty("/robosim/ini_pos_covar/x_std_dev_mm").doubleValue();
		if (props.containsProperty("/robosim/ini_pos_covar/y_std_dev_mm"))
			yStdDev = props.getDoubleProperty("/robosim/ini_pos_covar/y_std_dev_mm").doubleValue();
		if (props.containsProperty("/robosim/ini_pos_covar/theta_std_dev_graus"))
			thetaStdDev = props.getDoubleProperty("/robosim/ini_pos_covar/theta_std_dev_graus").doubleValue();

		AbstractDoubleSquareMatrix initialPositionCovar = new DoubleSquareMatrix(3);
/*
		initialPositionCovar.setElement(0, 0, 1);
		initialPositionCovar.setElement(1, 1, 1);
		initialPositionCovar.setElement(2, 2, 3*Math.PI/180);

		initialPositionCovar.setElement(0, 0, 120*120);
		initialPositionCovar.setElement(1, 1, 120*120);
		initialPositionCovar.setElement(2, 2, 1*Math.PI/180*1*Math.PI/180);
//*/
		initialPositionCovar.setElement(0, 0, xStdDev*xStdDev);
		initialPositionCovar.setElement(1, 1, yStdDev*yStdDev);
		initialPositionCovar.setElement(2, 2, thetaStdDev*thetaStdDev*Math.PI*Math.PI/(180*180));
		return initialPositionCovar;
	}


	public boolean validateWallReading(double xS, double yS, double thetaS, double thetaR, double reading, Wall wall, double sigmaPosition2, double sigmaReading2)
	{
		if (logModel.isDebugEnabled())
			logModel.debug("validateWallReading::sigmaReading2: " + sigmaReading2 + ", sigmaPosition2: " + sigmaPosition2);
		double sigmaError2 = sigmaPosition2 + sigmaReading2;
		double xSW = xS*wall.getCosTheta() + yS*wall.getSinTheta();

		double value = 0;

		if (FunctionsR.angularDist(thetaR + thetaS, wall.getTheta()) > Math.PI/2)
			value = Math.abs((xSW - wall.getD()) - reading);
		else
			value = Math.abs((wall.getD() - xSW) - reading);

		double zValue2 = value*value/sigmaError2;

		if (logModel.isDebugEnabled())
		{
			logModel.debug("xSW: " + xSW + ", wall.getD(): " + wall.getD() + ", reading: " + reading + ", sigmaError2: " + sigmaError2);
			logModel.debug("value: " + value + ", zValue2: " + zValue2 + ", wallRejectionValue2: " + wallRejectionValue2);
		}

		return (zValue2 < wallRejectionValue2*10);
	}

/*
	public List findWalls(double thetaMin, double thetaMax, double xS, double yS, double sigmaPosition2, double sigmaReading2)
	{
		ArrayList listAcceptedWalls = new ArrayList();
		double sigmaError2 = sigmaPosition2 + sigmaReading2;
		Iterator itWalls = this.findWalls(thetaMin, thetaMax, xS, yS).iterator();
		while (itWalls.hasNext())
		{
			Wall wall = (Wall) itWalls.next(); 
			double value = ((Double) itWalls.next()).doubleValue(); 
			double zValue2 = value*value/sigmaError2;
			if (logModel.isDebugEnabled())
				logModel.debug("value: " + value + ", zValue2: " + zValue2 + ", sigmaError2: " + sigmaError2 + ", wallRejectionValue2: " + wallRejectionValue2);
			if (zValue2 < wallRejectionValue2*100)
			{
				listAcceptedWalls.add(wall);
				listAcceptedWalls.add(new Double(zValue2));
			}
		}
		return listAcceptedWalls;
	}
//*/

	
	public List findWalls(double thetaMin, double thetaMax, double xS, double yS)
	{
		double thetaMean = (FunctionsR.shiftRadian2FirstLoop(thetaMin) + FunctionsR.shiftRadian2FirstLoop(thetaMax))/2;
		double cosThetaMean = Math.cos(thetaMean);
		double sinThetaMean = Math.sin(thetaMean);
		ArrayList listAcceptedWalls = new ArrayList();
		int [] arrayIdxMin = new int[4];
		double [] arrayThetaMin = new double[4], arrayThetaMax = new double[4];
		this.getThetaIndices(thetaMin, thetaMax, arrayIdxMin, arrayThetaMin, arrayThetaMax);

		if (logModel.isDebugEnabled())
			logModel.debug("findWalls:thetaMin : thetaMax -> " + thetaMin + " : " + thetaMax);
		for (int idxIdxMin = 0; arrayIdxMin[idxIdxMin] != -1; idxIdxMin++)
		{
			if (logModel.isDebugEnabled())
				logModel.debug("arrayIdxMin[idxIdxMin] -> " + arrayIdxMin[idxIdxMin]);
			for (int i = arrayIdxMin[idxIdxMin]; i < arrayOrderedTheta.length && arrayOrderedTheta[i] <= arrayThetaMax[idxIdxMin] + 0.001; i++)
			{
				Wall wall = arrayThetaOrderedWall[i];
				double xSW = xS*wall.getCosTheta() + yS*wall.getSinTheta();
				double thetaWall = wall.getTheta();
				if (!(Math.abs(xSW) < Math.abs(wall.getD()) || FunctionsR.sinalTrocado(xSW, wall.getD())))
					thetaWall -= Math.PI;
				thetaWall = FunctionsR.shiftRadian2FirstLoop(thetaWall); 
				if (logModel.isDebugEnabled())
					logModel.debug("xSW -> " + xSW + ", thetaWall -> " + thetaWall + ", wall.getD() -> " + wall.getD() + ", wall -> " + wall);
				if (!FunctionsR.inRange(thetaWall, thetaMin, thetaMax) && Math.abs(wall.getD()) > 0.1)
				{
					if (logModel.isDebugEnabled())
						logModel.debug("eliminado por if (thetaMin > thetaWall || thetaMax < thetaWall) && Math.abs(wall.getD()) > 0.1: " + wall);
					continue;
				}
				double value = 0;
				
				listAcceptedWalls.add(wall);
			}
		}
		return listAcceptedWalls;
	}


	public double findReadingToClosestCrossingWall(double thetaMin, double thetaMax, double x1, double y1, double x2, double y2)
	{
		int [] arrayIdxMin = new int[4];
		double [] arrayThetaMin = new double[4];
		double [] arrayThetaMax = new double[4];
		this.getThetaIndices(thetaMin, thetaMax, arrayIdxMin, arrayThetaMin, arrayThetaMax);

		if (logSens.isDebugEnabled())
			logSens.debug("findReadingToClosestCrossingWall:thetaMin : thetaMax -> " + thetaMin + " : " + thetaMax);

		Wall wallMin = null;
		double distMin = 10000000;
		for (int idxIdxMin = 0; arrayIdxMin[idxIdxMin] != -1; idxIdxMin++)
		{
			for (int i = arrayIdxMin[idxIdxMin]; i < arrayOrderedTheta.length && arrayOrderedTheta[i] <= arrayThetaMax[idxIdxMin] + 0.001; i++)
			{
				Wall wall = arrayThetaOrderedWall[i];
				double xSW = x1*wall.getCosTheta() + y1*wall.getSinTheta();
				double thetaWall = wall.getTheta();
				if (!(Math.abs(xSW) < Math.abs(wall.getD()) || FunctionsR.sinalTrocado(xSW, wall.getD())))
					thetaWall -= Math.PI;
				thetaWall = FunctionsR.shiftRadian2FirstLoop(thetaWall); 
				if (logSens.isDebugEnabled())
					logSens.debug("xSW -> " + xSW + ", thetaWall -> " + thetaWall + ", wall.getD() -> " + wall.getD() + ", wall -> " + wall);
				if (!FunctionsR.inRange(thetaWall, thetaMin, thetaMax))
				{
					if (logSens.isDebugEnabled())
						logSens.debug("melô, if (FunctionsR.inRange(thetaWall, thetaMin, thetaMax)): " + wall);
					continue;
				}
				
				if (GeomOperations.intersect(x1, y1, x2, y2, wall.getX1(), wall.getY1(), wall.getX2(), wall.getY2()))
				{			
					Point2D pInt = GeomOperations.intersection(x1, y1, x2, y2, wall.getX1(), wall.getY1(), wall.getX2(), wall.getY2());
				
					double dx = pInt.getX() - x1, dy = pInt.getY() - y1; 
					double dist = dx*dx + dy*dy;
	
					if (logSens.isDebugEnabled())
						logSens.debug("\twall: " + wall + ", pInt = " + pInt + ", dist = " + dist);
				
					if (dist < distMin)
					{
						distMin = dist;
						wallMin = wall;  
					}
				}
				else
				{
					if (logSens.isDebugEnabled())
						logSens.debug("\twall: " + wall + ", não intersecta");
				}
			}
		}

		if (wallMin != null)
		{
			double xSW = x1*wallMin.getCosTheta() + y1*wallMin.getSinTheta();
			return Math.abs(wallMin.getD() - xSW);
		}
		else
		{
			if (logSens.isDebugEnabled())
				logSens.debug("wall não encontrado!");
			return 10000000;
		}
	}


	/**
	 *  Testa só de 0 a 180. O teste de 0 a 360 é feito wall a wall.
	 * 
	 * @param thetaMin
	 * @param thetaMax
	 * @param arrayIdxMin
	 * @param arrayThetaMin
	 * @param arrayThetaMax
	 */
	private void getThetaIndices(double thetaMin, double thetaMax, int [] arrayIdxMin, double [] arrayThetaMin, double [] arrayThetaMax)
	{
		if (thetaMin > Math.PI)
			thetaMin -= Math.PI;
		if (thetaMin > Math.PI*2 && thetaMax > Math.PI*2)
		{
			thetaMin -= 2*Math.PI;
			thetaMax -= 2*Math.PI;
		}
		else if (thetaMax > Math.PI*2)
			thetaMax -= Math.PI;
		if (thetaMin < 0 && thetaMax < 0)
		{
			thetaMin += 2*Math.PI;
			thetaMax += 2*Math.PI;
		}

		int count = 0;
		if (thetaMin < 0)
		{ 
			arrayIdxMin[count] = arrayOrderedTheta.length/2;
			arrayThetaMin[count] = Math.PI + thetaMin;
			arrayThetaMax[count] = Math.PI;
			thetaMin = 0;
			count++;
		}
		if (thetaMin > Math.PI && thetaMax > Math.PI)
		{ 
			thetaMin -= Math.PI;
			thetaMax -= Math.PI;
		}
		else if (thetaMax > Math.PI)
		{ 
			arrayIdxMin[count] = arrayOrderedTheta.length/2;
			arrayThetaMin[count] = 0;
			arrayThetaMax[count] = thetaMax - Math.PI;
			thetaMax = Math.PI;
			count++;
		}
		arrayIdxMin[count] = arrayOrderedTheta.length/2;
		arrayThetaMin[count] = thetaMin;
		arrayThetaMax[count] = thetaMax;
		count++;
		arrayIdxMin[count] = -1;
		
		for (int i = 0; arrayIdxMin[i] != -1; i++)
		{
			int inc = (arrayOrderedTheta.length + 3)/4;
			while (true)
			{
				if (arrayOrderedTheta[arrayIdxMin[i]] >= arrayThetaMin[i])
				{
					if (arrayIdxMin[i] - inc >= 0)
					arrayIdxMin[i] -= inc;
					else if (arrayIdxMin[i] - 1 >= 0)
					arrayIdxMin[i] -= 1;
					else
						break;
				}
				else if (arrayOrderedTheta[arrayIdxMin[i]] < arrayThetaMin[i])
				{
					if (arrayIdxMin[i] + inc < arrayOrderedTheta.length)
					arrayIdxMin[i] += inc;
					else if (arrayIdxMin[i] + 1 < arrayOrderedTheta.length)
						arrayIdxMin[i] += 1;
					else
						break;
				}

				if (inc > 1)
					inc = (inc + 1)/2;
				else
				{
					while (arrayIdxMin[i] - 1 > -1 && arrayOrderedTheta[arrayIdxMin[i] - 1] >= arrayThetaMin[i])
					arrayIdxMin[i]--;
					while (arrayIdxMin[i] + 1 < arrayOrderedTheta.length && arrayOrderedTheta[arrayIdxMin[i]] < arrayThetaMin[i])
					arrayIdxMin[i]++;
					break;
				}
			}
			if (logSens.isDebugEnabled())
				logSens.debug("getThetaIndices:arrayIdxMin[i]:" + arrayIdxMin[i] + ", arrayThetaMin[i] = " + arrayThetaMin[i] + ", arrayThetaMax[i] = " + arrayThetaMax[i]);
		}
	}


	public List findLines(double xMin, double xMax, double yMin, double yMax)
	{
		ArrayList listAcceptedLines = new ArrayList();
		int idxY = this.getLineYFirstIndex(yMin);

		if (logModel.isDebugEnabled())
			logModel.debug("findLines:idxY -> " + idxY);

		for (int i = idxY; i < arrayOrderedLineY.length && arrayOrderedLineY[i] <= yMax + 0.001; i++)
		{
			VLineRef line = arrayOrderedLine[i];
			if (line.getX() >= xMin && line.getX() <= xMax)
			{
				listAcceptedLines.add(line);
			}
		}
		return listAcceptedLines;
	}


	/**
	 * Retorna lista de projeções esperadas ordenadas.
	 *
	 */
	public List getOrderedExpectedProjections(RigidBodyTranformation2D robotPose, CameraModel camModel, double extraViewFactor)
	{
		TreeSet setExpProjs = new TreeSet();

		double f = camModel.getUFocusDistance();
		double uSize = camModel.getUAxisPixelCount();
		double uSizeExtra = extraViewFactor*uSize;
		double uCenter = camModel.getUAxisPixelCenter();
		double uSizeExtraBand = uSizeExtra/2;
		double zRotCamera = camModel.getZRotCamera();
		RigidBodyTranformation2D camPose = robotPose.rot(zRotCamera);
		double rTheta = camPose.getAsPose().getTheta();

		for (int i = 0; i < arrayOrderedLine.length; i++)
		{
			VLineRef lineRef = arrayOrderedLine[i];
			AbstractDoubleVector invLine = camPose.inverseTrafoLine(lineRef);

			if (invLine.getComponent(0) > 0)
			{
				double thAng = FunctionsR.angularDist(lineRef.getNormalInv(), rTheta);
				if (thAng > Math.PI)
				{
					log.debug("getOrderedExpectedProjections:if (thAng > Math.PI): " + rTheta + ", line ref: " + lineRef);
					continue;
				}
				double uExpected = invLine.getComponent(1) * f / invLine.getComponent(0);
				log.debug("uExpected: " + uExpected + " for lineRef: " + lineRef);
				if (Math.abs(uExpected) < uSizeExtraBand)
				{
//					uExpected = uCenter - uExpected;
					uExpected = uCenter + uExpected;
					log.debug("accepted: " + uExpected);
					setExpProjs.add(new VLineRefProj(uExpected, lineRef));
				}
			}
		}
		logVisionMarcoCount.debug(setExpProjs.size() + ";");
		return new ArrayList(setExpProjs);
	}


	private int getLineYFirstIndex(double yMin)
	{
		int inc = (arrayOrderedLineY.length + 3)/4;
		int retVal = arrayOrderedLineY.length/2;
		while (true)
		{
			if (arrayOrderedLineY[retVal] >= yMin)
			{
				if (retVal - inc >= 0)
					retVal -= inc;
				else if (retVal - 1 >= 0)
					retVal -= 1;
				else
					break;
			}
			else if (arrayOrderedLineY[retVal] < yMin)
			{
				if (retVal + inc < arrayOrderedLineY.length)
				retVal += inc;
				else if (retVal + 1 < arrayOrderedLineY.length)
					retVal += 1;
				else
					break;
			}

			if (inc > 1)
				inc = (inc + 1)/2;
			else
			{
				while (retVal - 1 > -1 && arrayOrderedLineY[retVal - 1] >= yMin)
					retVal--;
				while (retVal + 1 < arrayOrderedLineY.length && arrayOrderedLineY[retVal] < yMin)
					retVal++;
				break;
			}
		}

		return retVal;
	}
}

//1234567890123456789012345
//1234567890123456789012345
