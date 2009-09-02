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
public class V2LineProjPairGenerator implements LineProjPairGenerator
{
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	private static Log log = LogFactory.getLog(V2LineProjPairGenerator.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");

	// Parâmetros
	private double hashVectorTreshold = 50;
	private boolean bUseWindow = true;
	private boolean bUseDirectionWindow = false;
	private int directWindowSize = 60;


	public V2LineProjPairGenerator()
	{
	}


	public void setUseWindow(boolean bVal)	{this.bUseWindow = bVal;}
	public void setUseDirectionWindow(boolean bVal)	{this.bUseDirectionWindow = bVal;}
	public void setDirectWindowSize(int val)	{this.directWindowSize = val;}
	public void setHashVectorTreshold(double hashVectorTreshold) {this.hashVectorTreshold = hashVectorTreshold;}


	public PairedLines createLineProjPairs(RigidBodyTranformation2D cameraTrafo, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar, LineExtrationResult newLineProjs, LineMatchingResult lineMatchingResult, AbstractDoubleSquareMatrix cameraPoseCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap, DifferenceSimpleMatchingPhaseMatcher callback)
	{
		PairedLines result = new PairedLines();
		TreeMap mapLastMeasuredProjectionOrdered = lineMap.buildLastMeasuredProjectionOrderedMap();
		result.arrayProjLastMeasuresAlreadyUsed = new boolean[mapLastMeasuredProjectionOrdered.keySet().size()];
		result.arrayProjLastMeasuresLines = new VLine[mapLastMeasuredProjectionOrdered.keySet().size()];
		result.arrayProjMeasuresAlreadyUsed = new boolean[newLineProjs.arrayProjMeasures.length];
		result.arrayTreeMapLinks = new TreeMap[mapLastMeasuredProjectionOrdered.keySet().size()];

		RigidBodyTranformation2D cameraMoveTrafo = new RigidBodyTranformation2D(cameraMoveEstimate);

		/*PRINTIMGS
		log.debug("[in] cameraTrafo: " + cameraTrafo);
		log.debug("[in] cameraMoveTrafo: " + cameraMoveTrafo);
		//*/

		int uCount = camModel.getUAxisPixelCount(), uCenter = camModel.getUAxisPixelCenter();
		double visAngle = camModel.getVisibilityRangeAngle();
		double f = camModel.getUFocusDistance();

		// rotation window
		// Possíveis variações na orientação do robô.
		double thetaMult = uCount/visAngle;
		double rotWindowCenter = (cameraMoveEstimate.getTheta()*thetaMult);
//		double rotWindowSideBandSize = (Math.sqrt(cameraMoveCovar.getElement(2, 2))*thetaMult*2);
		double rotWindowSideBandSize = 3.0*Math.PI/180.0*thetaMult*2;
		if (rotWindowSideBandSize < uCount/40) rotWindowSideBandSize = uCount/40;
		/*PRINTIMGS
		log.debug("rotWindowCenter = " + rotWindowCenter + ", rotWindowSideBandSize = " + rotWindowSideBandSize + ", thetaMult = " + thetaMult);
		log.debug("cameraMoveCovar: \r\n" + MatrixUtil.toString(cameraMoveCovar, 9, 4));
		//*/

		// Possíveis variações na posição do robô.
		AbstractDoubleVector maxDisp = cameraTrafo.inverseTrafoPointMaxDisp(cameraMoveCovar);
		double xDisp = Math.abs(maxDisp.getComponent(0)), yDisp = Math.abs(maxDisp.getComponent(1));
		AbstractDoubleVector maxDispPose = cameraTrafo.inverseTrafoPointMaxDisp(cameraPoseCovar);
		double xDispPose = Math.abs(maxDispPose.getComponent(0)), yDispPose = Math.abs(maxDispPose.getComponent(1));

		int idxBeginSearch = 0, idxEndSearch = 0, idxLastLineIdx = -1, desempate = 0;

		// Itera pelas últimas projeções associadas às retas verticais
		// sendo observadas.
		Iterator itLastMeasuredProjections = mapLastMeasuredProjectionOrdered.keySet().iterator();
		if (newLineProjs.arrayProjMeasures.length > 0) while (itLastMeasuredProjections.hasNext())
		{
			Object oKey = itLastMeasuredProjections.next();
			VLine line = (VLine) mapLastMeasuredProjectionOrdered.get(oKey);
			VLineProj lineLastProjFromVLine = lineMap.getLastMeasuredProjection(line);
			VLinePerfil linePerfil = lineLastProjFromVLine.getPerfil();

			idxLastLineIdx++;
			result.arrayTreeMapLinks[idxLastLineIdx] = new TreeMap();
			result.arrayProjLastMeasuresLines[idxLastLineIdx] = line;

			int lastVBegin = lineLastProjFromVLine.getVBegin(), lastVEnd = lineLastProjFromVLine.getVEnd();
			double uLastMeasured = lineLastProjFromVLine.getU();
			double uSigmaLastMeasured = lineLastProjFromVLine.getUErrorSigma();

			// Cálculo da janela de visualização
			double measureWindowBegin = -1, measureWindowEnd = -1;
			log.debug("translaction window for: u = " + uLastMeasured + ", projection of: " + line);

			// Se já existir mapeado uma reta para a projeção ...
			/*PRINTIMGS
			log.debug("isMapped: " + lineMap.isLineMapMeasure(line));
			//*/
			if (lineMap.isLineMapMeasure(line))
			{
				VLineRef lineRef = lineMap.getAssocVLineRef(line);
				AbstractDoubleVector lineRelative = cameraTrafo.inverseTrafoLine(lineRef);
				/*PRINTIMGS
				log.debug("lineRef: " + lineRef);
				log.debug("lineRelative: " + lineRelative);
				//*/

				// Linha fora do raio de visão do robô (10 centímetros da câmera) ...
				if (lineRelative.getComponent(0) < 1)
				{
					log.debug("FORA DO CAMPO DE VISÃO (para trás da câmera)");

					// invalida linha?
					
					continue;
				}

				double lineProjMean = uCenter + f * lineRelative.getComponent(1) / lineRelative.getComponent(0);
				/*PRINTIMGS
				log.debug("lineProjMean: " + lineProjMean);
				log.debug("uCenter + f * lineRelative.getComponent(1) / lineRelative.getComponent(0): " + (uCenter + f * lineRelative.getComponent(1) / lineRelative.getComponent(0)));
				log.debug("uCenter - f * lineRelative.getComponent(1) / lineRelative.getComponent(0): " + (uCenter - f * lineRelative.getComponent(1) / lineRelative.getComponent(0)));
				log.debug("-uCenter + f * lineRelative.getComponent(1) / lineRelative.getComponent(0): " + (-uCenter + f * lineRelative.getComponent(1) / lineRelative.getComponent(0)));
				//*/

				// Da um chorinho ...
				double xDispLinePos = xDispPose + 100;
				double yDispLinePos = yDispPose + 10;
				/*PRINTIMGS
				log.debug("xDispLinePos = " + xDispLinePos + ", yDispLinePos = " + yDispLinePos);
				//*/

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
				double wndSize = Math.sqrt(windowX*windowX + windowY*windowY);

				measureWindowBegin = (lineProjMean - wndSize/2.0);
				measureWindowEnd = (0.5 + lineProjMean + wndSize/2.0);
				
				/*PRINTIMGS
				log.debug("wndSize: " + wndSize + ", windowX: " + windowX + ", windowY: " + windowY + ", lineProjMean: " + lineProjMean);
				log.debug("lineProjs: " + lineProjs[0] + ", " + lineProjs[1] + ", " + lineProjs[2] + ", " + lineProjs[3]);
				log.debug("BEFORE ROTATION WND: " + measureWindowBegin + " --  " + measureWindowEnd);
				//*/

				measureWindowBegin -= rotWindowSideBandSize;
				measureWindowEnd += rotWindowSideBandSize;
//					measureWindowBegin += rotWindowCenter;
//					measureWindowEnd += rotWindowCenter;
				log.debug("AFTER ROTATION WND:  " + measureWindowBegin + " --  " + measureWindowEnd);

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
				log.debug("COXA FINAL:          " + measureWindowBegin + " --  " + measureWindowEnd);
			}
			else 
			{
				double uLast = lineLastProjFromVLine.getU();
				double xLastFarRelative = 25000;
				double xLastNearRelative =  350 + cameraMoveTrafo.getAsPose().getX();
				double yLastFarRelative = (uCenter - uLast) * xLastFarRelative / f;
				double yLastNearRelative = (uCenter - uLast) * xLastNearRelative / f;

				double [] lineActualFar = new double[2];
				cameraMoveTrafo.inverseLine(xLastFarRelative, yLastFarRelative, lineActualFar);
				double [] lineActualNear = new double[2];
				cameraMoveTrafo.inverseLine(xLastNearRelative, yLastNearRelative, lineActualNear);
				double uNear = uCenter - lineActualNear[1] * f / lineActualNear[0];
				double uFar = uCenter - lineActualFar[1] * f / lineActualFar[0];

				/*PRINTIMGS
				log.debug("uLast: " + uLast + ", uCenter: " + uCenter + "(uCenter - uLast): " + (uCenter - uLast) + ", (uLast - uCenter): " + (uLast - uCenter));
				log.debug("yLastFarRelative: " + yLastFarRelative + ", xLastFarRelative: " + xLastFarRelative);
				log.debug("yLastNearRelative: " + yLastNearRelative + ", xLastNearRelative: " + xLastNearRelative);
				log.debug("far:  " + lineActualFar[0] + ", " + lineActualFar[1]);
				log.debug("near: " + lineActualNear[0] + ", " + lineActualNear[1]);
				log.debug("uFar: " + uFar + ", uNear: " + uNear);
				log.debug("lineActualFar[1] * f / lineActualFar[0]: " + (lineActualFar[1] * f / lineActualFar[0]) + "lineActualNear[1] * f / lineActualNear[0]: " + (lineActualNear[1] * f / lineActualNear[0]));
				//*/

				if (uFar > uNear)
				{
					measureWindowBegin = uNear;
					measureWindowEnd = uFar;
				}
				else
				{
					measureWindowBegin = uFar;
					measureWindowEnd = uNear;
				}
				log.debug("BEFORE ROTATION WND: " + measureWindowBegin + " --  " + measureWindowEnd);
				measureWindowBegin -= rotWindowSideBandSize;
				measureWindowEnd += rotWindowSideBandSize;
				log.debug("AFTER ROTATION WND:  " + measureWindowBegin + " --  " + measureWindowEnd);
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

			/*PRINTIMGS
			log.debug("idxBeginSearch = " + idxBeginSearch + ", idxEndSearch = " + idxEndSearch);
			log.debug("newLineProjs.arrayProjMeasures.length = " + newLineProjs.arrayProjMeasures.length);
			//*/

			// Em seguida verifica dentro da janela quem tem condições de corresponder.
			for (int idx = idxBeginSearch; idx < idxEndSearch; idx++)
			{
//				int vBegin = newLineProjs.arrayProjMeasures[idx].getVBegin(), vEnd = newLineProjs.arrayProjMeasures[idx].getVEnd();
//				int vSize = vEnd - vBegin + 1;

				if (true)
//				if ((vBegin <= lastVBegin && lastVBegin <= vEnd) || (lastVBegin <= vBegin && vBegin <= lastVEnd))
				{
					double comparsion = linePerfil.compare(newLineProjs.arrayProjMeasures[idx].getPerfil());
					double comparsionFirstHalf = linePerfil.compareFirstHalf(newLineProjs.arrayProjMeasures[idx].getPerfil());
					double comparsionSecondHalf = linePerfil.compareSecondHalf(newLineProjs.arrayProjMeasures[idx].getPerfil());

					/*PRINTIMGS
					log.debug("|||perfilR 1: " + Arrays.toString(newLineProjs.arrayProjMeasures[idx].getPerfil().getPerfilRed(), newLineProjs.arrayProjMeasures[idx].getPerfil().getPerfilRed().length, 3));
					log.debug("|||perfilR 2: " + Arrays.toString(linePerfil.getPerfilRed(), newLineProjs.arrayProjMeasures[idx].getPerfil().getPerfilRed().length, 3));
					log.debug("comparsion: " + comparsion + ", comparsionFirstHalf: " + comparsionFirstHalf + ", comparsionSecondHalf: " + comparsionSecondHalf);
					//*/

					int idxsPacked = (idxLastLineIdx&0xFFFF)<<16 | (idx&0xFFFF);
					if (comparsion > hashVectorTreshold || 
						comparsionFirstHalf > hashVectorTreshold || 
						comparsionSecondHalf > hashVectorTreshold)
					{
						/*PRINTIMGS
						log.debug("u last: " + uLastMeasured + ",u: " + newLineProjs.arrayProjMeasures[idx].getU());
						//*/
						PerfilComparator compa = new PerfilComparator(comparsion, comparsionFirstHalf, comparsionSecondHalf, desempate++);
						result.arrayTreeMapLinks[idxLastLineIdx].put(compa, new Integer(idxsPacked));
						result.treeMapLinks.put(compa, new Integer(idxsPacked));
					}
					/*PRINTIMGS
					else
					{
						log.debug("u last: " + uLastMeasured + ",u: " + newLineProjs.arrayProjMeasures[idx].getU() + "; not matched by perfil");
					}
					//*/
				}
				/*PRINTIMGS
				else
				{
					double comparsion = linePerfil.compare(newLineProjs.arrayProjMeasures[idx].getPerfil());
					log.debug("comparsion = " + comparsion + ",u last: " + uLastMeasured + ",u: " + newLineProjs.arrayProjMeasures[idx].getU() + "; not matched by geom contraints");
				}
				//*/
			}
			log.debug("result.arrayTreeMapLinks[" + idxLastLineIdx + "].size() = " + result.arrayTreeMapLinks[idxLastLineIdx].size());
		}
		else while (itLastMeasuredProjections.hasNext())
		{
			idxLastLineIdx++;
			Object oKey = itLastMeasuredProjections.next();
			VLine line = (VLine) mapLastMeasuredProjectionOrdered.get(oKey);
			VLineProj lineProj = lineMap.getLastMeasuredProjection(line);
			VLinePerfil linePerfil = lineProj.getPerfil();
			result.arrayProjLastMeasuresLines[idxLastLineIdx] = line;
		}
		return result;
	}
}

