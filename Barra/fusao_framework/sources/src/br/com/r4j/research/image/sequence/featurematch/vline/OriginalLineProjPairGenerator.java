package br.com.r4j.research.image.sequence.featurematch.vline;

import java.text.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;
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
public class OriginalLineProjPairGenerator implements LineProjPairGenerator
{
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	private static Log log = LogFactory.getLog(OriginalLineProjPairGenerator.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");

	// Parâmetros
	private double hashVectorTreshold = 50;
	private boolean bUseWindow = true;
	private boolean bUseDirectionWindow = false;
	private int directWindowSize = 60;


	public OriginalLineProjPairGenerator()
	{
	}


	public void setUseWindow(boolean bVal)	{this.bUseWindow = bVal;}
	public void setUseDirectionWindow(boolean bVal)	{this.bUseDirectionWindow = bVal;}
	public void setDirectWindowSize(int val)	{this.directWindowSize = val;}
	public void setHashVectorTreshold(double hashVectorTreshold) {this.hashVectorTreshold = hashVectorTreshold;}


	public PairedLines createLineProjPairs(RigidBodyTranformation2D cameraTrafo, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar, LineExtrationResult newLineProjs, LineMatchingResult lineMatchingResult, AbstractDoubleSquareMatrix cameraPoseCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap, DifferenceSimpleMatchingPhaseMatcher callback)
	{
		PairedLines result = new PairedLines();
		RigidBodyTranformation2D cameraMoveTrafo = new RigidBodyTranformation2D(cameraMoveEstimate);

		log.debug("[in] cameraTrafo: " + cameraTrafo);
		log.debug("[in] cameraMoveTrafo: " + cameraMoveTrafo);

		int uCount = camModel.getUAxisPixelCount(), uCenter = camModel.getUAxisPixelCenter();
		double visAngle = camModel.getVisibilityRangeAngle();
		double f = camModel.getUFocusDistance();

		// rotation window
		double thetaMult = uCount/visAngle;
		double rotWindowCenter = (cameraMoveEstimate.getTheta()*thetaMult);

		// 2 é o fator de incerteza.
		double rotWindowSideBandSize = (Math.sqrt(cameraMoveCovar.getElement(2, 2))*thetaMult*2);
		if (rotWindowSideBandSize < uCount/115) rotWindowSideBandSize = uCount/115;

		log.debug("rotWindowCenter = " + rotWindowCenter + ", rotWindowSideBandSize = " + rotWindowSideBandSize + ", thetaMult = " + thetaMult);
		log.debug("cameraMoveCovar: \r\n" + MatrixUtil.toString(cameraMoveCovar, 9, 4));

		TreeMap treeMap = lineMap.buildLastMeasuredProjectionOrderedMap();
		result.arrayProjLastMeasuresAlreadyUsed = new boolean[treeMap.keySet().size()];
		result.arrayProjLastMeasuresLines = new VLine[treeMap.keySet().size()];
		result.arrayProjMeasuresAlreadyUsed = new boolean[newLineProjs.arrayProjMeasures.length];
		result.arrayTreeMapLinks = new TreeMap[treeMap.keySet().size()];

		int idxBeginSearch = 0, idxEndSearch = 0, idxLastLineIdx = -1, desempate = 0;

		AbstractDoubleVector maxDisp = cameraTrafo.inverseTrafoPointMaxDisp(cameraMoveCovar);
		double xDisp = Math.abs(maxDisp.getComponent(0)), yDisp = Math.abs(maxDisp.getComponent(1));

		AbstractDoubleVector maxDispPose = cameraTrafo.inverseTrafoPointMaxDisp(cameraPoseCovar);
		double xDispPose = Math.abs(maxDispPose.getComponent(0)), yDispPose = Math.abs(maxDispPose.getComponent(1));

		Iterator itElements = treeMap.keySet().iterator();
		if (newLineProjs.arrayProjMeasures.length > 0) while (itElements.hasNext())
		{
			idxLastLineIdx++;
			result.arrayTreeMapLinks[idxLastLineIdx] = new TreeMap();
			Object oKey = itElements.next();
			VLine line = (VLine) treeMap.get(oKey);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);
			VLinePerfil linePerfil = lineProj.getPerfil();
			result.arrayProjLastMeasuresLines[idxLastLineIdx] = line;

			int lastVBegin = lineProj.getVBegin(), lastVEnd = lineProj.getVEnd();
			double uLastMeasured = lineProj.getU();
			double uSigmaLastMeasured = lineProj.getUErrorSigma();

			// Cálculo da janela de visualização
			double measureWindowBegin = -1, measureWindowEnd = -1;
			log.debug("translaction window for: u = " + uLastMeasured + ", projection of: " + line);

			if (bUseWindow || bUseDirectionWindow)
			{
				// Se já existir mapeado uma reta para a projeção ...
				log.debug("hasModel: " + lineMap.hasLineModel(line));
				log.debug("isMapped: " + lineMap.isLineMapMeasure(line));
				boolean bHasLineModel = lineMap.hasLineModel(line);
				if (lineMap.isLineMapMeasure(line))
				{
					VLineRef lineRef = lineMap.getAssocVLineRef(line);
					log.debug("lineRef: " + lineRef);
					AbstractDoubleVector lineRelative = cameraTrafo.inverseTrafoLine(lineRef);
					log.debug("cameraTrafo: " + cameraTrafo);
					log.debug("lineRelative: " + lineRelative);

					// Linha fora do rai de visão do robô (10 centímetros da câmera) ...
					if (lineRelative.getComponent(0) < 1)
					{
						// invalida linha?
						continue;
					}

					double lineProjMean = (uCenter - f*(lineRelative.getComponent(1)) / lineRelative.getComponent(0));
					log.debug("lineProjMean: " + lineProjMean);

					double xDispLinePos = xDispPose + 100;
					double yDispLinePos = yDispPose + 10;
					log.debug("xDispLinePos = " + xDispLinePos + ", yDispLinePos = " + yDispLinePos);

					double [] lineProjs = new double[4];
					lineProjs[0] = (double) ((lineRelative.getComponent(1) - yDispLinePos) / lineRelative.getComponent(0));
					lineProjs[1] = (double) ((lineRelative.getComponent(1) + yDispLinePos) / lineRelative.getComponent(0));
					if (lineRelative.getComponent(0) - xDispLinePos <= 100)
						lineProjs[2] = (double) ((lineRelative.getComponent(1)) / (100));
					else
						lineProjs[2] = (double) ((lineRelative.getComponent(1)) / (lineRelative.getComponent(0) - xDispLinePos));
					lineProjs[3] = (double) ((lineRelative.getComponent(1)) / (lineRelative.getComponent(0) + xDispLinePos));

					double windowX = (f*Math.abs(lineProjs[1] - lineProjs[0]));
					double windowY = (f*Math.abs(lineProjs[3] - lineProjs[2]));

					log.debug("xDispPose = " + xDispPose + ", yDispPose = " + yDispPose);
					log.debug("windowX = " + windowX + ", windowY = " + windowY + ", lineProjMean: " + lineProjMean);
					log.debug("lineProjs: " + lineProjs[0] + ", " + lineProjs[1] + ", " + lineProjs[2] + ", " + lineProjs[3]);

					double wndSize = Math.sqrt(windowX*windowX + windowY*windowY);

					measureWindowBegin = (lineProjMean - wndSize/2.0);
					measureWindowEnd = (0.5 + lineProjMean + wndSize/2.0);

					measureWindowBegin -= rotWindowSideBandSize;
					measureWindowEnd += rotWindowSideBandSize;
//					measureWindowBegin += rotWindowCenter;
//					measureWindowEnd += rotWindowCenter;

					if (lineProjMean - uLastMeasured > 0)
					{
						if (measureWindowBegin > uLastMeasured - 2)
							measureWindowBegin = uLastMeasured - 2;
					}
					else
					{
						if (measureWindowEnd < uLastMeasured + 2)
							measureWindowEnd = uLastMeasured + 2;
					}
				}
				else 
				{
					if (bHasLineModel)
					{
						VLine lineWrk = (VLine) lineMap.getLineModel(line);
						log.debug("lineWrk: " + lineWrk);

						AbstractDoubleVector lineRelative = null;
						if (lineMap.isLineState(line)) 
						{
							log.debug("state already: " + lineWrk);

							VLineXY lineWrkModel = new VLineXY(lineWrk.getRho()*lineWrk.getCosBeta(), lineWrk.getRho()*lineWrk.getSinBeta(), f);
							lineRelative = cameraTrafo.inverseTrafoLine(lineWrkModel);
							log.debug("cameraTrafo: " + cameraTrafo);
						}
						else
						{
							VLineXY lineWrkModel = (VLineXY) lineWrk;
							RigidBodyTranformation2D trafoModel = lineMap.getLineModelRigidBodyTranformation2D(line);
							log.debug("trafoModel: " + trafoModel);
							AbstractDoubleVector lineWrkAbs = trafoModel.directTrafoLine(lineWrkModel);
							log.debug("lineWrkAbs: " + MatrixUtil.toString(lineWrkAbs, 7, 3));
							log.debug("cameraTrafo: " + cameraTrafo);
							lineRelative = cameraTrafo.inverseTrafoLine(lineWrkAbs);
						}

						log.debug("lineRelative: " + lineRelative);

						// primeiro rotaciona, calculando a posícão média da reta rotacionada, e depois
						// calcula a janela de deslocamento sobre ssa nova pos.
						//

						// Linha fora do rai de visão do robô (10 centímetros da câmera) ...
						if (lineRelative.getComponent(0) < 1)
						{
							bHasLineModel = false;
							lineMap.invalidateModel(line);
						}
						else
						{
							double lineProjMean = (uCenter - f*(lineRelative.getComponent(1)) / lineRelative.getComponent(0));
							log.debug("lineProjMean: " + lineProjMean);
							if (bUseDirectionWindow)
							{
								measureWindowBegin = uLastMeasured + rotWindowCenter - rotWindowSideBandSize;
								measureWindowEnd = uLastMeasured + rotWindowCenter + rotWindowSideBandSize;
								if (lineProjMean - uLastMeasured > 0)
									measureWindowEnd += directWindowSize;
								else
									measureWindowBegin -= directWindowSize;
							}
							else
							{
								double xDispLinePos = xDisp + 1000;
								double yDispLinePos = yDisp + 100;

								// se a linah já for estado ...
								if (lineWrk.getCovar() != null)
								{
									log.debug("lineWrk.getCovar(): " + MatrixUtil.toString(lineWrk.getCovar(), 7, 3));
									xDispLinePos = xDisp + Math.sqrt(lineWrk.getCovar().getElement(0, 0));
									yDispLinePos = yDisp + Math.sqrt(lineWrk.getCovar().getElement(1, 1));
								}
								log.debug("xDispLinePos = " + xDispLinePos + ", yDispLinePos = " + yDispLinePos);

								double [] lineProjs = new double[4];
								lineProjs[0] = (double) ((lineRelative.getComponent(1) - yDispLinePos) / lineRelative.getComponent(0));
								lineProjs[1] = (double) ((lineRelative.getComponent(1) + yDispLinePos) / lineRelative.getComponent(0));
								if (lineRelative.getComponent(0) - xDispLinePos <= 100)
									lineProjs[2] = (double) ((lineRelative.getComponent(1)) / (100));
								else
									lineProjs[2] = (double) ((lineRelative.getComponent(1)) / (lineRelative.getComponent(0) - xDispLinePos));
								lineProjs[3] = (double) ((lineRelative.getComponent(1)) / (lineRelative.getComponent(0) + xDispLinePos));

								double windowX = (f*Math.abs(lineProjs[1] - lineProjs[0]));
								double windowY = (f*Math.abs(lineProjs[3] - lineProjs[2]));

								log.debug("xDisp = " + xDisp + ", yDisp = " + yDisp);
								log.debug("windowX = " + windowX + ", windowY = " + windowY + ", lineProjMean: " + lineProjMean);
								log.debug("lineProjs: " + lineProjs[0] + ", " + lineProjs[1] + ", " + lineProjs[2] + ", " + lineProjs[3]);

								double wndSize = Math.sqrt(windowX*windowX + windowY*windowY);

								measureWindowBegin = (lineProjMean - wndSize/2);
								measureWindowEnd = (lineProjMean + wndSize/2);

								measureWindowBegin -= rotWindowSideBandSize;
								measureWindowEnd += rotWindowSideBandSize;
							}
							if (lineProjMean - uLastMeasured > 0)
							{
								if (measureWindowBegin > uLastMeasured - 2)
									measureWindowBegin = uLastMeasured - 2;
							}
							else
							{
								if (measureWindowEnd < uLastMeasured + 2)
									measureWindowEnd = uLastMeasured + 2;
							}
						}
					}
					if (!bHasLineModel)
					{
						measureWindowBegin = uLastMeasured - rotWindowSideBandSize;
						measureWindowEnd = uLastMeasured + rotWindowSideBandSize;

						double guessedLastY_p1 = (uCenter - uLastMeasured) * 500 / f;
						double guessedRecentY_p2 = cameraMoveTrafo.inverseTrafoLineGetY(1.0*500, guessedLastY_p1);
						log.debug("guessedLastY_p1 = " + guessedLastY_p1 + ", guessedRecentY_p2 = " + guessedRecentY_p2 + ", cameraMoveEstimate: " + cameraMoveEstimate);

						if (guessedRecentY_p2 - guessedLastY_p1 > 0)
							measureWindowEnd += directWindowSize;
						else
							measureWindowBegin -= directWindowSize;
					}
			}
			}
			else 
			{
				measureWindowBegin = -camModel.getUAxisPixelCount();
				measureWindowEnd = camModel.getUAxisPixelCount()*2;
			}
			log.debug("measureWindowBegin = " + measureWindowBegin + ", measureWindowEnd = " + measureWindowEnd);

			// Correção par evitar janelas muito grandes.
			if (uLastMeasured - measureWindowBegin > uCount/6)
				measureWindowBegin = uLastMeasured - uCount/6;
			if (measureWindowEnd - uLastMeasured > uCount/6)
				measureWindowEnd = uLastMeasured + uCount/6;
			log.debug("[after correction] measureWindowBegin = " + measureWindowBegin + ", measureWindowEnd = " + measureWindowEnd);

			// Posiciona a janela de medição.
			while (idxBeginSearch > 0 && newLineProjs.arrayProjMeasures[idxBeginSearch].getU() >= measureWindowBegin) 
				idxBeginSearch--;
			while (newLineProjs.arrayProjMeasures[idxBeginSearch].getU() < measureWindowBegin && idxBeginSearch < newLineProjs.arrayProjMeasures.length - 1) 
				idxBeginSearch++;

			if (idxEndSearch > newLineProjs.arrayProjMeasures.length - 1)
				idxEndSearch = newLineProjs.arrayProjMeasures.length - 1;
			while (idxEndSearch > 0 && newLineProjs.arrayProjMeasures[idxEndSearch].getU() > measureWindowEnd) 
				idxEndSearch--;
			while (newLineProjs.arrayProjMeasures[idxEndSearch].getU() <= measureWindowEnd && idxEndSearch < newLineProjs.arrayProjMeasures.length - 1) 
				idxEndSearch++;
			if (newLineProjs.arrayProjMeasures[idxEndSearch].getU() <= measureWindowEnd) 
				idxEndSearch++;

			log.debug("idxBeginSearch = " + idxBeginSearch + ", idxEndSearch = " + idxEndSearch);
			log.debug("newLineProjs.arrayProjMeasures.length = " + newLineProjs.arrayProjMeasures.length);


			// Em seguida verifica dentro da janela quem tem condições de corresponder.
			for (int idx = idxBeginSearch; idx < idxEndSearch; idx++)
			{
				int vBegin = newLineProjs.arrayProjMeasures[idx].getVBegin(), vEnd = newLineProjs.arrayProjMeasures[idx].getVEnd();
				int vSize = vEnd - vBegin + 1;

				if (true)
//				if ((vBegin <= lastVBegin && lastVBegin <= vEnd) || (lastVBegin <= vBegin && vBegin <= lastVEnd))
				{
					log.debug("|||perfilR 1: " + Arrays.toString(newLineProjs.arrayProjMeasures[idx].getPerfil().getPerfilRed(), newLineProjs.arrayProjMeasures[idx].getPerfil().getPerfilRed().length, 3));
					log.debug("|||perfilR 2: " + Arrays.toString(linePerfil.getPerfilRed(), newLineProjs.arrayProjMeasures[idx].getPerfil().getPerfilRed().length, 3));
					double comparsion = linePerfil.compare(newLineProjs.arrayProjMeasures[idx].getPerfil());
					double comparsionFirstHalf = linePerfil.compareFirstHalf(newLineProjs.arrayProjMeasures[idx].getPerfil());
					double comparsionSecondHalf = linePerfil.compareSecondHalf(newLineProjs.arrayProjMeasures[idx].getPerfil());
					log.debug("comparsion: " + comparsion + ", comparsionFirstHalf: " + comparsionFirstHalf + ", comparsionSecondHalf: " + comparsionSecondHalf);

					int idxsPacked = (idxLastLineIdx&0xFFFF)<<16 | (idx&0xFFFF);
					if (comparsion > hashVectorTreshold || 
						comparsionFirstHalf > hashVectorTreshold || 
						comparsionSecondHalf > hashVectorTreshold)
					{
						log.debug("u last: " + uLastMeasured + ",u: " + newLineProjs.arrayProjMeasures[idx].getU());
						PerfilComparator compa = new PerfilComparator(comparsion, comparsionFirstHalf, comparsionSecondHalf, desempate++);
						result.arrayTreeMapLinks[idxLastLineIdx].put(compa, new Integer(idxsPacked));
						result.treeMapLinks.put(compa, new Integer(idxsPacked));
					}
					else
					{
						log.debug("u last: " + uLastMeasured + ",u: " + newLineProjs.arrayProjMeasures[idx].getU() + "; not matched by perfil");
					}
				}
				else
				{
					double comparsion = linePerfil.compare(newLineProjs.arrayProjMeasures[idx].getPerfil());
					log.debug("comparsion = " + comparsion + ",u last: " + uLastMeasured + ",u: " + newLineProjs.arrayProjMeasures[idx].getU() + "; not matched by geom contraints");
				}
			}
			log.debug("result.arrayTreeMapLinks[" + idxLastLineIdx + "].size() = " + result.arrayTreeMapLinks[idxLastLineIdx].size());
		}
		else while (itElements.hasNext())
		{
			idxLastLineIdx++;
			Object oKey = itElements.next();
			VLine line = (VLine) treeMap.get(oKey);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);
			VLinePerfil linePerfil = lineProj.getPerfil();
			result.arrayProjLastMeasuresLines[idxLastLineIdx] = line;
		}
		return result;
	}
}

