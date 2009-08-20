package br.com.r4j.robosim;

import java.awt.geom.*;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.Shape;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.EventListenerList;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
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

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;



/**
 * 
 * @author giord
 *
 * O mapa é formado por um conjunto de paredes dispostas em qualquer posição, além de um conjunto 
 * de pontos que indicam retas visuais identificáveis no ambiente, além das quinas das paredes. 
 *
 * arquivo do mapa:
 *
 * mov:distancia(mm):velocidade(mm/it):angulo(graus):velocidade(graus/it)
 */
public class WorldMap extends TokenLineStreamParser implements RendererListener
{
	private static Log log = LogFactory.getLog(WorldMap.class.getName());
	private static Log logSens = LogFactory.getLog("sonar");
	private static Log logModel = LogFactory.getLog("sonarmodel");

	private ArrayList listPolygons = new ArrayList();
	private List listSegs = new ArrayList();
	private ArrayList listPointsVLine = new ArrayList();

	private IPolygon polyOuter = null;

	private double [] arrayOrderedTheta = null;
	private Wall [] arrayThetaOrderedWall = null;

	private EventListenerList listenerList = null;
	private File flMapFile = null;

	private String strNameMyName = null;

	private double wallRejectionValue = 1.645;
	private double wallRejectionValue2 = wallRejectionValue*wallRejectionValue;

	private int xIni = -1;
	private int yIni = -1;


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
		this.flMapFile = fFile;
		ArrayList listLists = new ArrayList();
		int width = -1, height = -1;
		xIni = -1; yIni = -1;

		char [] arrayDataBuffer = new char[200];
		String strWord = null;
		log.debug("start parse:");
		try
		{
			FileReader fRead = new FileReader(flMapFile);

			strWord = this.getWord(fRead, arrayDataBuffer);
			while (strWord != null)
			{
				log.debug("strWord:" + strWord);
				if (strWord.startsWith(";"))
				{
					this.jumpToNextLine(fRead, arrayDataBuffer);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("width"))
				{
					width = this.getInt(fRead, arrayDataBuffer);
					log.debug("width = " + width);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("height"))
				{
					height = this.getInt(fRead, arrayDataBuffer);
					log.debug("height = " + height);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("position"))
				{
					xIni = this.getInt(fRead, arrayDataBuffer);
					yIni = this.getInt(fRead, arrayDataBuffer);
					log.debug("xIni = " + xIni + ", yIni = " + yIni + ", theta = " + strWord);
					strWord = this.getWord(fRead, arrayDataBuffer);
					strWord = this.getWord(fRead, arrayDataBuffer);
				}
				else if (strWord.equalsIgnoreCase("vline"))
				{
					int xPoint = this.getInt(fRead, arrayDataBuffer);
					int yPoint = this.getInt(fRead, arrayDataBuffer);
					listPointsVLine.add(new Point(xPoint, yPoint));
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
			log.debug("end parse");

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
				log.debug("polyTmp:" + polyTmp);
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
			log.debug("listPolygons.size() = " + listPolygons.size());

			Iterator itVLines = listPointsVLine.iterator();
			while (itVLines.hasNext())
			{
				Point ptVLine = (Point) itVLines.next();
//				ptVLine.translate(-xIni, -yIni);
				
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
					logSens.debug("arrayThetaOrderedWall[" + counterTheta + "]: " + arrayThetaOrderedWall[counterTheta]);
					logSens.debug("arrayOrderedTheta[" + counterTheta + "]: " + arrayOrderedTheta[counterTheta]);
					counterTheta++;
				}
				
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
			System.err.println("Não foi possível caregar o mapa!");
			return;
		}
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
			RenderablePolygonInfo renderPolyInfo = new RenderablePolygonInfo(true, false, Color.white, Color.black, strokeObjectOutline, poly);
			renderPolyInfo.draw(g2d);
		}
//		g2d.setComposite(cOri);

		strokeObjectOutline = new BasicStroke(2f);
		RenderablePolygonInfo renderPolyInfoOutline = new RenderablePolygonInfo(true, false, Color.white, Color.black, strokeObjectOutline, polyOuter);
		renderPolyInfoOutline.draw(g2d);

		Shape shpCircle = ShapesUtil.createCircle(6);
//		shpCircle =  e.untransform(shpCircle);
		BasicStroke strokeVLines = new BasicStroke(1f);
		g2d.setStroke(strokeVLines);
		Iterator itPointsVLine = listPointsVLine.iterator();
		while (itPointsVLine.hasNext())
		{
			Point2D vline = (Point2D) itPointsVLine.next();
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


	public AbstractDoubleSquareMatrix getInitialLocalizationCovar()
	{
		AbstractDoubleSquareMatrix initialPositionCovar = new DoubleSquareMatrix(3);
		initialPositionCovar.setElement(0, 0, 0.01);
		initialPositionCovar.setElement(1, 1, 0.01);
		initialPositionCovar.setElement(2, 2, 0.0001);
		return initialPositionCovar;
	}



	public boolean validateWallReading(double xS, double yS, double thetaS, double thetaR, double reading, Wall wall, double sigmaPosition2, double sigmaReading2)
	{
		logModel.debug("validateWallReading::sigmaReading2: " + sigmaReading2 + ", sigmaPosition2: " + sigmaPosition2);
		double sigmaError2 = sigmaPosition2 + sigmaReading2;
		double xSW = xS*wall.getCosTheta() + yS*wall.getSinTheta();

		double value = 0;
/*		
		if (Math.abs(xSW) < Math.abs(wall.getD()) || FunctionsR.sinalTrocado(xSW, wall.getD()))
			value = Math.abs(xSW + reading - wall.getD());
		else
			value = Math.abs(xSW - reading - wall.getD());
//*/			
		if (FunctionsR.angularDist(thetaR + thetaS, wall.getTheta()) > Math.PI/2)
			value = Math.abs((xSW - wall.getD()) - reading);
		else
			value = Math.abs((wall.getD() - xSW) - reading);

		double zValue2 = value*value/sigmaError2;

		logModel.debug("xSW: " + xSW + ", wall.getD(): " + wall.getD() + ", reading: " + reading + ", sigmaError2: " + sigmaError2);
		logModel.debug("value: " + value + ", zValue2: " + zValue2 + ", wallRejectionValue2: " + wallRejectionValue2);

		return (zValue2 < wallRejectionValue2*10);
	}


	public List findWalls(double thetaMin, double thetaMax, double xS, double yS, double reading, double sigmaPosition2, double sigmaReading2)
	{
		ArrayList listAcceptedWalls = new ArrayList();
		double sigmaError2 = sigmaPosition2 + sigmaReading2;
		Iterator itWalls = this.findWalls(thetaMin, thetaMax, xS, yS, reading).iterator();
		while (itWalls.hasNext())
		{
			Wall wall = (Wall) itWalls.next(); 
			double value = ((Double) itWalls.next()).doubleValue(); 
			double zValue2 = value*value/sigmaError2;
			logModel.debug("value: " + value + ", zValue2: " + zValue2 + ", sigmaError2: " + sigmaError2 + ", wallRejectionValue2: " + wallRejectionValue2);
			if (zValue2 < wallRejectionValue2*100)
			{
				listAcceptedWalls.add(wall);
				listAcceptedWalls.add(new Double(zValue2));
			}
		}
		return listAcceptedWalls;
	}

	
	public List findWalls(double thetaMin, double thetaMax, double xS, double yS, double reading)
	{
		ArrayList listAcceptedWalls = new ArrayList();

		int [] arrayIdxMin = new int[4];
		double [] arrayThetaMin = new double[4];
		double [] arrayThetaMax = new double[4];
		this.getThetaIndices(thetaMin, thetaMax, arrayIdxMin, arrayThetaMin, arrayThetaMax);

		logModel.debug("findWalls:thetaMin : thetaMax -> " + thetaMin + " : " + thetaMax);
		
		for (int idxIdxMin = 0; arrayIdxMin[idxIdxMin] != -1; idxIdxMin++)
		{
			logModel.debug("arrayIdxMin[idxIdxMin] -> " + arrayIdxMin[idxIdxMin]);
			for (int i = arrayIdxMin[idxIdxMin]; i < arrayOrderedTheta.length && arrayOrderedTheta[i] <= arrayThetaMax[idxIdxMin] + 0.001; i++)
			{
				Wall wall = arrayThetaOrderedWall[i];
				double xSW = xS*wall.getCosTheta() + yS*wall.getSinTheta();
				double thetaWall = wall.getTheta();
				if (!(Math.abs(xSW) < Math.abs(wall.getD()) || FunctionsR.sinalTrocado(xSW, wall.getD())))
					thetaWall -= Math.PI;
				thetaWall = FunctionsR.shiftRadian2FirstLoop(thetaWall); 
				logModel.debug("xSW -> " + xSW + ", thetaWall -> " + thetaWall + ", wall.getD() -> " + wall.getD() + ", wall -> " + wall);
				if (!FunctionsR.inRange(thetaWall, thetaMin, thetaMax))
				{
					logModel.debug("eliminado por if (thetaMin > thetaWall || thetaMax < thetaWall): " + wall);
					continue;
				}

				double value = 0;				
//				if (Math.abs(xSW) < Math.abs(wall.getD()) || FunctionsR.sinalTrocado(xSW, wall.getD()))
				if (FunctionsR.angularDist((thetaMin + thetaMax)/2, wall.getTheta()) > Math.PI/2)
					value = Math.abs((xSW - wall.getD()) - reading);
				else
					value = Math.abs((wall.getD() - xSW) - reading);
	
				logModel.debug("value: " + value);
				listAcceptedWalls.add(wall);
				listAcceptedWalls.add(new Double(value));
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
				logSens.debug("xSW -> " + xSW + ", thetaWall -> " + thetaWall + ", wall.getD() -> " + wall.getD() + ", wall -> " + wall);
				if (!FunctionsR.inRange(thetaWall, thetaMin, thetaMax))
				{
					logSens.debug("melô, if (FunctionsR.inRange(thetaWall, thetaMin, thetaMax)): " + wall);
					continue;
				}
				
				if (GeomOperations.intersect(x1, y1, x2, y2, wall.getX1(), wall.getY1(), wall.getX2(), wall.getY2()))
				{			
					Point2D pInt = GeomOperations.intersection(x1, y1, x2, y2, wall.getX1(), wall.getY1(), wall.getX2(), wall.getY2());
				
					double dx = pInt.getX() - x1, dy = pInt.getY() - y1; 
					double dist = dx*dx + dy*dy;
	
					logSens.debug("\twall: " + wall + ", pInt = " + pInt + ", dist = " + dist);
				
					if (dist < distMin)
					{
						distMin = dist;
						wallMin = wall;  
					}
				}
				else
				{
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
			logSens.debug("wall não encontrado!");
			return 10000000;
		}
//		return Math.sqrt(distMin);
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
			logSens.debug("getThetaIndices:arrayIdxMin[i]:" + arrayIdxMin[i] + ", arrayThetaMin[i] = " + arrayThetaMin[i] + ", arrayThetaMax[i] = " + arrayThetaMax[i]);
		}
	}
}



