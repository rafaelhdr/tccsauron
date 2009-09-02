package br.com.r4j.research.image.sequence.featurematch.vline;

import java.text.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;
import java.awt.Color;
import java.awt.image.BufferedImage;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.research.vline.*;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.WorldMap;
import br.com.r4j.commons.util.*;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;


/**
 *
 */
public class DifferenceSimpleMatchingPhaseMatcher
{
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	private static Log log = LogFactory.getLog(DifferenceSimpleMatchingPhaseMatcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");

	private static int MIN_PROJ_GATE_ACCEPTANCE = 40;

	///// Variáveis com a descrição do sistema
	//
	private CameraModel camModel = null;

	///// Variáveis com o estado do sistema
	//
	private VLineMap lineMap = null;

	///// Variáveis com o mapa conhecido
	//
	private WorldMap worldMap = null;


	private LineMapMatcher lineMapMatcher = null;
	private LineProjMatcher lineProjMatcher = null;
	private LineProjPairGenerator lineProjPairGenerator = null;


	// Parâmetros
	private double hashVectorTreshold = 50;
	private int minDepthDist = 3;
	private boolean bUseWindow = true;
	private boolean bUseDirectionWindow = false;
	private int directWindowSize = 60;
	private boolean bSimpleMatch = false;

	private int [] arrayOutDebug = null;



	public DifferenceSimpleMatchingPhaseMatcher()
	{
//		lineMapMatcher = new V1MapMatcher();
		lineMapMatcher = new V2MapMatcher();

//		lineProjMatcher = new SimpleLineProjMatcher();
		lineProjMatcher = new ComplexV1LineProjMatcher();

//		lineProjPairGenerator = new OriginalLineProjPairGenerator();
		lineProjPairGenerator = new V2LineProjPairGenerator();
	}


	public void setUseWindow(boolean bVal)
	{
		this.bUseWindow = bVal;
		lineProjPairGenerator.setUseWindow(bVal);
	}
	
	public void setSimpleMatch(boolean bVal)
	{
		this.bSimpleMatch = bVal;
	}
	
	public void setUseDirectionWindow(boolean bVal)
	{
		this.bUseDirectionWindow = bVal;
		lineProjPairGenerator.setUseDirectionWindow(bVal);
	}
	
	public void setDirectWindowSize(int val)
	{
		this.directWindowSize = val;
		lineProjPairGenerator.setDirectWindowSize(val);
	}

	public void setMinDepth(int minDepth)	{this.minDepthDist = minDepth;}
	public void setHashVectorTreshold(double hashVectorTreshold)	
	{
		this.hashVectorTreshold = hashVectorTreshold;
		lineProjMatcher.setHashVectorTreshold(hashVectorTreshold);
		lineProjPairGenerator.setHashVectorTreshold(hashVectorTreshold);
	}

	public void setCameraModel(CameraModel camModel)	{this.camModel = camModel;}
	public void setLineMap(VLineMap lineMap)	{this.lineMap = lineMap;}
	public void setWorldMap(WorldMap worldMap)	{this.worldMap = worldMap;}
	public void setItCount(int itCount)	{this.itCount = itCount;}

	private int itCount = 0; // de-de-debug!


	private int imgWidth = -1, imgHeight = -1;
	private LineMatchingResult result = null;
	public LineMatchingResult lineMatching(int [] arrayImg, int imgWidth, int imgHeight, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar, LineExtrationResult newLineProjs, boolean bMapMatch)
	{
		LineMatchingResult result = new LineMatchingResult();
		result.arrayProjMeasuresLines = new VLine[newLineProjs.arrayProjMeasures.length];
		long timeTaken = System.currentTimeMillis();

		/*PRINTIMGS
		log.debug("cameraMoveEstimate = " + cameraMoveEstimate);
		//*/
		if (cameraMoveEstimate != null)
		{
			PairedLines pairedLines = lineProjPairGenerator.createLineProjPairs(cameraTrafo, cameraMoveEstimate, cameraMoveCovar, newLineProjs, result, cameraPoseCovar, camModel, lineMap, worldMap, this);

			// procura pelos melhores matches ...
			lineProjMatcher.performMatches(newLineProjs, pairedLines, result, cameraTrafo, cameraPoseCovar, cameraMoveEstimate, cameraMoveCovar, camModel, lineMap, worldMap, this);

			// Adiciona as medidas não casadas como novas estimativas não válidas.
			this.addNotMatchedMeasures(pairedLines.arrayProjMeasuresAlreadyUsed, newLineProjs.arrayProjMeasures, result.arrayProjMeasuresLines, cameraTrafo, cameraPoseCovar);

			// Elimina as linhas de matches não bem-sucedidos.
			this.removeNotMatchedLines(pairedLines.arrayProjLastMeasuresAlreadyUsed, pairedLines.arrayProjLastMeasuresLines);
		}
		else
		{
			// Adiciona as medidas não casadas como novas estimativas não válidas.
			this.addNotMatchedMeasures(null, newLineProjs.arrayProjMeasures, result.arrayProjMeasuresLines, cameraTrafo, cameraPoseCovar);
		}

		if (bMapMatch)
			this.matchWithWorldMap(newLineProjs, cameraTrafo, cameraPoseCovar);

		/*PRINTIMGS
		if (log.isDebugEnabled())
		{
			if (arrayOutDebug == null)
				arrayOutDebug = new int[arrayImg.length];
			br.com.r4j.commons.util.Arrays.arrayCopy(arrayImg, arrayOutDebug);
			this.imgWidth = imgWidth;
			this.imgHeight = imgHeight;
			this.result = result;

			log.debug("lineMap.getNumberOfLines() = " + lineMap.getNumberOfLines());

			TreeMap treeMap = lineMap.buildLastMeasuredProjectionOrderedMap();
			Iterator itElements = treeMap.keySet().iterator();
			log.debug("["+itCount+"]output:treeMap.keySet().size() = " + treeMap.keySet().size());
			while (itElements.hasNext())
			{
				boolean bIsNewLine = false;
				Object oKey = itElements.next();
				VLine line = (VLine) treeMap.get(oKey);
				VLineProj lineProjBefore = lineMap.getBeforeLastMeasuredProjection(line);
				VLineProj lineProj = lineMap.getLastMeasuredProjection(line);

				Color clr = null;
				clr = lineMap.getDebugColor(line);
				int clrVal = 0xFFFFFF;
				if (clr == null)
				{
					bIsNewLine = true;
					clr = ColorSequence.getNonWhiteNonBlackColor(lineMap.nextColorIdx++, 0);
					lineMap.setDebugColor(line, clr);
				}
				clrVal = ThreeBandPackedUtil.getPackedPixel(clr);

				if (lineProjBefore != null)
					log.debug("output:lineProj.getU() = " + lineProj.getU() + ", lineMap.arrayTTL[line.mapIdx] = " + lineMap.arrayTTL[line.mapIdx] + " -> " + lineProjBefore.getU() + ", new? [" + bIsNewLine + "], clrVal: " + clrVal);
				else
					log.debug("output:lineProj.getU() = " + lineProj.getU() + ", lineMap.arrayTTL[line.mapIdx] = " + lineMap.arrayTTL[line.mapIdx] + " -> NULL" + ", new? [" + bIsNewLine + "], clrVal: " + clrVal);

				ThreeBandPackedUtil.fillRect(lineProj.getSegment().xIni - 1, lineProj.getSegment().yIni - 1, 2, 2, 0x00FF00, arrayOutDebug, imgWidth, imgHeight);
				ThreeBandPackedUtil.fillRect(lineProj.getSegment().xEnd - 1, lineProj.getSegment().yEnd - 1, 2, 2, 0xFF0000, arrayOutDebug, imgWidth, imgHeight);
				ThreeBandPackedUtil.plotLine(lineProj.getSegment().xIni, lineProj.getSegment().yIni,  lineProj.getSegment().xEnd, lineProj.getSegment().yEnd, 
												clrVal, arrayOutDebug, imgWidth, imgHeight);
				if (bIsNewLine)
				{
					ThreeBandPackedUtil.fillRect(lineProj.getSegment().xIni - 3, (lineProj.getSegment().yEnd + lineProj.getSegment().yIni)/2 - 3, 6, 6, 0x00FFFF, arrayOutDebug, imgWidth, imgHeight);
				}

				ThreeBandPackedUtil.plotString(lineProj.getSegment().xEnd + 5, lineProj.getSegment().yEnd, 
													0xFF00FF, "" + numFrmt.format(lineProj.getU()), arrayOutDebug, imgWidth, imgHeight);
				if (lineProjBefore != null)
					ThreeBandPackedUtil.plotString(lineProj.getSegment().xEnd + 5, lineProj.getSegment().yIni + 15, 
													0xFFFF00, "" + numFrmt.format(lineProjBefore.getU()), arrayOutDebug, imgWidth, imgHeight);
			}
			if (result.arrayProjMeasuresLines != null)
				log.debug("arrayProjMeasuresLines = " + br.com.r4j.commons.util.Arrays.toString(result.arrayProjMeasuresLines));
			ImageUtil.saveImageBMP(ImageUtil.createBufferedImage(arrayOutDebug,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "debA_02_" + itCount + ".bmp");
		}
		//*/

		return result;
	}


	private Pose2D poseActualRobot = null;
	public Pose2D poseLastRobot = null;

	private double [] arrayLineBuffer = new double[2];
	public void updateFutureLineStates(LineMatchingResult matchings, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix cameraMoveCovar) 
	{ 
		if (log.isDebugEnabled())
			log.debug("updateFutureLineStates: " + matchings.arrayProjMeasuresLines.length);

		poseLastRobot = poseActualRobot;
		poseActualRobot = cameraTrafo.getAsPose();

		double f = camModel.getUFocusDistance();
		double uCenter = camModel.getUAxisPixelCenter();
		for (int i = 0; i < matchings.arrayProjMeasuresLines.length; i++) 
		{ 
			VLine vline = matchings.arrayProjMeasuresLines[i]; 
			log.debug("vline: " + vline);
			if (lineMap.isLineState(vline)) 
			{
				if (log.isDebugEnabled())
					log.debug("is state: " + lineMap.getLineModel(vline));
				continue;
			}

			if (lineMap.isLineMapMeasure(vline)) 
			{
				if (log.isDebugEnabled())
					log.debug("is map measure: " + lineMap.getLineModel(vline));
				continue;
			}

			// Tenta melhorar o modelo. 
			// Verifica se ele tem condições de ser um estado. se sim, marca como o sendo. 
			if (!lineMap.hasLineModel(vline)) 
			{
				if (log.isDebugEnabled())
					log.debug("Uma medição. Que beleza:" + lineMap.getLastMeasuredProjection(vline).getU());
				VLineXY lineEst = new VLineXY(3000, (uCenter - lineMap.getLastMeasuredProjection(vline).getU())*3000/f, f);
				if (log.isDebugEnabled())
					log.debug("lineEst: " + lineEst);
/*
				cameraTrafo.directLine(lineEst.getX(), lineEst.getY(), arrayLineBuffer);
				log.debug("lineEst.directLine: " + arrayLineBuffer[0] + ", " + arrayLineBuffer[1]);
				double rho = Math.sqrt(arrayLineBuffer[0]*arrayLineBuffer[0] + arrayLineBuffer[1]*arrayLineBuffer[1]);
				double beta = Math.asin(arrayLineBuffer[1]/rho);
//*/
				double rho = Math.sqrt(lineEst.getX()*lineEst.getX() + lineEst.getY()*lineEst.getY());
				double beta = Math.asin(lineEst.getY()/rho);
				if (log.isDebugEnabled())
					log.debug("rho: " + rho + ", beta: " + beta +  " (" + (beta*180/Math.PI) + ") graus");
				VLine lineNew = new VLine(rho, beta, f);
				AbstractDoubleSquareMatrix iniCovar = new DoubleSquareMatrix(3);
				iniCovar.setElement(0, 0, 4000);
//				iniCovar.setElement(0, 0, 10000);
//				iniCovar.setElement(1, 1, cameraPoseCovar.getElement(2, 2));
//				iniCovar.setElement(1, 1, 0.001);
//				iniCovar.setElement(1, 1, 0.008);
				iniCovar.setElement(1, 1, 0.01);
				lineMap.setBeforeLastMeasuredRigidBodyTranformation2D(vline, cameraTrafo);
				lineMap.setBeforeLastMeasuredProjection(vline, lineMap.getLastMeasuredProjection(vline));
				lineMap.setLineAsState(vline, lineNew, iniCovar, cameraTrafo);
			} 
			// não faz nada ... 
			else 
			{
				if (log.isDebugEnabled())
					log.debug("Shouldn't be here!!");
			}
		} 
	} 


	/**
	 * Organiza as coisas para o caso de match das linhas
	 *
	 * Calcula o pior caso do depth. Isso é para a menor variação do movimento do robô e maior deslocamento ds pixels.
	 */
	public void projectionMatched(LineMatch lineMatch, VLinePerfil measuredPerfil, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix cameraMoveCovar)
	{
		VLine line = lineMatch.line; 
		VLineProj projMeasured = lineMatch.projNew;

		if (log.isDebugEnabled())
			log.debug("old band: " + lineMap.getMatchedBand(line) + ",new band: " + lineMatch.matchedBand);
		lineMap.setMatchedBand(line, lineMatch.matchedBand);

		lineMap.setBeforeLastMeasuredProjection(line, lineMap.getLastMeasuredProjection(line));
		lineMap.setMeasuredProjection(line, projMeasured, cameraTrafo.getAsPose(), cameraPoseCovar, cameraTrafo);
		lineMap.arrayTTL[lineMatch.line.mapIdx]++;
	}


	/**
	 * Adiciona projeções encontradas na iteração mas não associadas a nenhuma 
	 * retas sendo rastreada.
	 */
	private void addNotMatchedMeasures(boolean [] arrayProjMeasuresAlreadyUsed, VLineProj [] arrayProjMeasures, VLine [] arrayProjMeasuresLines, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar)
	{
		int uCenter = camModel.getUAxisPixelCenter();
		for (int i = 0; i < arrayProjMeasures.length; i++)
		{
			if (arrayProjMeasuresAlreadyUsed == null || !arrayProjMeasuresAlreadyUsed[i])
			{
//				double uValue = arrayProjMeasures[i].getU();

				// O erro inicial não importa.
//				AbstractDoubleSquareMatrix covar = new DoubleSquareMatrix(2); 
//				covar.setElement(0, 0, 10000*10000);
//				covar.setElement(1, 1, 1.5*1.5);

				// Linha inútil. Serve só para iniciar.
//				VLine line = new VLine(minDepthDist, (uValue - uCenter));
				VLine line = new VLine(1, 1, 1);

				// A linha agora é considerada em relação a primeira posição de câmera onde foi capturada.
				// O erro deve representrar isso.
//				cameraTrafo.inverseTrafoLineByRef(line);

				arrayProjMeasuresLines[i] = line;
				lineMap.add(line, arrayProjMeasures[i], cameraTrafo.getAsPose(), cameraPoseCovar, cameraTrafo);
				lineMap.arrayTTL[line.mapIdx] = 1;
				lineMap.setMatchedBand(line, VLineMap.NO_BAND);
			}
		}
	}


	private void removeNotMatchedLines(boolean [] arrayProjLastMeasuresAlreadyUsed, VLine [] arrayProjLastMeasuresLines)
	{
		if (log.isDebugEnabled())
			log.debug("remove:lineMap.getNumberOfStateLines(): " + lineMap.getNumberOfStateLines());
		for (int idx = 0; idx < arrayProjLastMeasuresAlreadyUsed.length; idx++)
		{
			if (!arrayProjLastMeasuresAlreadyUsed[idx])
			{
				VLine line = arrayProjLastMeasuresLines[idx];
				lineMap.arrayTTL[line.mapIdx] = 0;
				lineMap.setDebugColor(line, null);
				/*PRINTIMGS
				log.debug("removing :" + line);
				log.debug("\t:last proj: " + lineMap.getLastMeasuredProjection(line));
				//*/
				lineMap.remove(line);
			}
		}
		if (log.isDebugEnabled())
			log.debug("remove:lineMap.getNumberOfStateLines(): " + lineMap.getNumberOfStateLines());
	}


	boolean [] arrayMatchesExists = new boolean[100];
	LineProjMatchPair [] arrayMatches = new LineProjMatchPair[100];
	/**
	 * Considera que newLineProjs.arrayProjMeasures está ordenado.
	 *
	 */
	private void matchWithWorldMap(LineExtrationResult newLineProjs, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar)
	{
		lineMapMatcher.matchWithWorldMap(newLineProjs, cameraTrafo, cameraPoseCovar, camModel, lineMap, worldMap);
	}
}

