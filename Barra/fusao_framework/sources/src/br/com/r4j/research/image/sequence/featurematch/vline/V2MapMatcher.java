package br.com.r4j.research.image.sequence.featurematch.vline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;
import java.util.HashSet;
import java.text.*;

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
import br.com.r4j.graph.*;
import br.com.r4j.graph.INode;


/**
 *
 */
public class V2MapMatcher implements LineMapMatcher
{
	private static Log log = LogFactory.getLog(V2MapMatcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");
	private static Log logVisionMapPairsCount = LogFactory.getLog("pares_mapa_visao");
	private static Log logVisionMapPairs = LogFactory.getLog("pares_mapa_visao_u");
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}


	public V2MapMatcher()
	{
	}


	LineProjMatchPair [] arrayMatches = new LineProjMatchPair[1000];
	boolean [] arrayMatchesExists = new boolean[1000];
	double [] arrayMatchesUShiftOrdered = new double[1000];
	int [] arrayMatchesUShiftOrderedIndex = new int[1000];

	public double [] arrayLimitsFixos = {1, 2, 3, 5, 7, 9, 13, 17, 21};
	public double [] arrayLimitsRel = {0.05, 0.1, 0.15, 0.3};


	/**
	 * Considera que newLineProjs.arrayProjMeasures está ordenado.
	 *
	 */
	public void matchWithWorldMap(LineExtrationResult newLineProjs, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap)
	{
		//*PRINTIMGS
		log.debug("--------------------------------------------------------");
		log.debug("------------                    matchWithWorldMap   ----");
		log.debug("cameraTrafo: " + cameraTrafo);
		//*/

		int countCount = 0;
		String strPairs = "";

		// 1 - Realiza o pareamento entre linhas do mapa e projeções realizadas.
		List listExpectedProjsOrdered = worldMap.getOrderedExpectedProjections(cameraTrafo, camModel, 2);
		//*PRINTIMGS
		log.debug("expectes projs: " + listExpectedProjsOrdered);
		log.debug("measured projs: " + Arrays.toString(newLineProjs.arrayProjMeasures));
		//*/
		if (listExpectedProjsOrdered.size() == 0 || newLineProjs.arrayProjMeasures.length == 0)
		{
			//*PRINTIMGS
			log.debug("newLineProjs.arrayProjMeasures.length: " + newLineProjs.arrayProjMeasures.length);
			//*/
			for (int measIdx = 0; measIdx < newLineProjs.arrayProjMeasures.length; measIdx++)
			{
				VLineProj lMeasProj = newLineProjs.arrayProjMeasures[measIdx];
				VLine line = lineMap.getVLine4LastMeasuredProjection(lMeasProj);
				//*PRINTIMGS
				log.debug("lMeasProj: " + lMeasProj + ", line.mapIdx: " + line.mapIdx + ", lMeasProj.mapIdx: " + lMeasProj.mapIdx);
				log.debug("DUMPING REFS");
				//*/
				lineMap.unmatchWithMap(line);
			}
			logVisionMapPairsCount.debug("0;");
			logVisionMapPairs.debug(";");
			lineMap.unmatchAllWithMap();
			return;
		}

		// Arruma tamanho das matrizes
		int stride = newLineProjs.arrayProjMeasures.length;
		int sizeExpected = listExpectedProjsOrdered.size();
		int size = stride*sizeExpected;
		if (arrayMatches.length < size)
		{
			arrayMatches = new LineProjMatchPair[size + 10];
			arrayMatchesExists = new boolean[size + 10];
			arrayMatchesUShiftOrdered = new double[size + 10];
			arrayMatchesUShiftOrderedIndex = new int[size + 10];

		}

		// 1-a - Cria matriz booleana indicando quais cruzamentos são matches.
		//     - Adiciona os uShifts para serem ordenados.
		// O índice: id_projecao + stride*id_marco
		Iterator itExpectedProjs = listExpectedProjsOrdered.iterator();
		int countValid = 0;
		for (int expectedIdx = 0; expectedIdx < sizeExpected; expectedIdx++)
		{
			VLineRefProj lRefProj = (VLineRefProj) itExpectedProjs.next();
			for (int measIdx = 0; measIdx < newLineProjs.arrayProjMeasures.length; measIdx++)
			{
				VLineProj lMeasProj = newLineProjs.arrayProjMeasures[measIdx];

				double comp = lMeasProj.getPerfil().compare(lRefProj.getLineRef());
				if (comp > 0.5)
				{
					arrayMatches[measIdx + stride*expectedIdx] = new LineProjMatchPair(lRefProj, lMeasProj, comp);
					arrayMatchesExists[measIdx + stride*expectedIdx] = true;
					arrayMatchesUShiftOrdered[countValid] = lRefProj.getU() - lMeasProj.getU();
					arrayMatchesUShiftOrderedIndex[countValid] = measIdx + stride*expectedIdx;
					countValid++;
					//*PRINTIMGS
					log.debug("(" + expectedIdx + ", " + measIdx + ") pair: " + arrayMatches[measIdx + stride*expectedIdx]);
					//*/
				}
				else
				{
					//*PRINTIMGS
					log.debug(lRefProj + " - " + comp + " - " + lMeasProj + ": stopped by color");
					//*/
					arrayMatchesExists[measIdx + stride*expectedIdx] = false;
				}
			}
		}
		//*PRINTIMGS
		log.debug("countValid: " + countValid + ", stride: " + stride);
		//*/
		if (countValid < 3)
		{
			logVisionMapPairsCount.debug("0;");
			logVisionMapPairs.debug(";");
			lineMap.unmatchAllWithMap();
			return;
		}

		
		// 1-b - Orderna associações válidas por uShift
		Arrays.sort(arrayMatchesUShiftOrdered, arrayMatchesUShiftOrderedIndex, 0, countValid);
		//*PRINTIMGS
		log.debug("arrayMatchesUShiftOrdered: " + Arrays.toString(arrayMatchesUShiftOrdered, 1, 10, 2, countValid));
		//*/

		// 1-c - Caminha pelo vetor de uShift procurando pelas janelas de similaridade.
		ArrayList listAllPaths = new ArrayList();
		for (int idxUShiftFirst = 0; idxUShiftFirst < countValid; idxUShiftFirst++)
		{
			int lastLimSize = 1;
			boolean bFirstTime = true;

			// Janela de tamanho fixo.
			for (int iLims = 0; iLims < arrayLimitsFixos.length; iLims++)
			{
				for (int idxUShiftLast = 0; idxUShiftLast < countValid; idxUShiftLast++)
				{
					if (arrayMatchesUShiftOrdered[idxUShiftLast] - arrayMatchesUShiftOrdered[idxUShiftFirst] > arrayLimitsFixos[iLims])
					{
						// Pois já saiu da janela desejada. Volta um.
						idxUShiftLast--;

						// Se tiver mais que mínimo para gerar três associações.
						if (idxUShiftLast - idxUShiftFirst > lastLimSize)
						{
							lastLimSize = idxUShiftLast - idxUShiftFirst;
							List listPaths = this.getPossiblePaths(listExpectedProjsOrdered, arrayMatchesUShiftOrdered, arrayMatchesUShiftOrderedIndex, arrayMatches, idxUShiftFirst, idxUShiftLast, !bFirstTime, stride);
							listAllPaths.addAll(listPaths);
							bFirstTime = listPaths.size() > 0 | bFirstTime;
						}

						break;
					}
				}
			}
/*
			// Janela de tamanho relativo.
			for (int iLims = 0; iLims < arrayLimitsRel.length; iLims++)
			{
				for (int idxUShiftLast = 0; idxUShiftLast < countValid; idxUShiftLast++)
				{
					double maxMax = Math.abs(arrayMatchesUShiftOrdered[idxUShiftLast]);
					if (Math.abs(arrayMatchesUShiftOrdered[idxUShiftFirst]) > maxMax)
						maxMax = Math.abs(arrayMatchesUShiftOrdered[idxUShiftFirst]);
					if (arrayMatchesUShiftOrdered[idxUShiftLast] - arrayMatchesUShiftOrdered[idxUShiftFirst] > arrayLimitsRel[iLims]*maxMax)
					{
						// Pois já saiu da janela desejada. Volta um.
						idxUShiftLast--;

						// Se tiver mais que mínimo para gerar três associações.
						if (idxUShiftLast - idxUShiftFirst > lastLimSize)
						{
							lastLimSize = idxUShiftLast - idxUShiftFirst;
							List listPaths = this.getPossiblePaths(listExpectedProjsOrdered, arrayMatchesUShiftOrdered, arrayMatchesUShiftOrderedIndex, arrayMatches, idxUShiftFirst, idxUShiftLast, !bFirstTime, stride);
							listAllPaths.addAll(listPaths);
							bFirstTime = listPaths.size() > 0 | bFirstTime;
						}

						break;
					}
				}
			}
//*/
		}

		Iterator itPaths = null;
		//*PRINTIMGS
		log.debug("---------------- todos os caminhos:");
		itPaths = listAllPaths.iterator();
		while (itPaths.hasNext())
		{
			ListHead lstNodes = (ListHead) itPaths.next();
			log.debug("lstNodes: " + lstNodes);
		}
		log.debug("----------------  ----------------");
		//*/

		// Rankeia cada sequencia e determina a com maior pontuação.
		double uCenter = camModel.getVAxisPixelCount();
		TreeMap mapProjSequenceSegmentMetrics = new TreeMap();
		itPaths = listAllPaths.iterator();
		while (itPaths.hasNext())
		{
			ListHead lstNodes = (ListHead) itPaths.next();
			ListHead lstNodes_ = lstNodes;

			double uShift = 0, sumComp = 10000000, minU = 10000, maxU = -10000, sumCompMean = 0;
			double uShiftSum = 0, uShiftSqrSum = 0;
			double firstSideSign = 0, secSideSign = 1;
			int countPairs = 0;
			while (lstNodes != null)
			{
				LineProjMatchPair pair = lstNodes.pair;
				INode node = lstNodes.node;

				uShift += pair.getUShift();
				uShiftSum += pair.getUShift();
				uShiftSqrSum += pair.getUShift()*pair.getUShift();
				countPairs++;
				if (pair.getComp() < sumComp)
					sumComp = pair.getComp();
				sumCompMean += pair.getComp();
//				sumComp *= pair.getComp();
				if (minU > pair.getMeasProj().getU())
					minU = pair.getMeasProj().getU();
				if (maxU < pair.getMeasProj().getU())
					maxU = pair.getMeasProj().getU();

				lstNodes = lstNodes.prev;
			}
			if (countPairs > 0)
			{
				uShift /= countPairs;
				sumCompMean /= countPairs;
			}

			double uVar = uShiftSqrSum/countPairs - uShift*uShift;

			//*PRINTIMGS
			log.debug("uVar: " + uVar + ", uShift: " + uShift + ", minU: " + minU + ", maxU: " + maxU + ", countPairs: " + countPairs + ", sumComp: " + sumComp);
			//*/

//			LineProjSequenceSegmentMetric metric = new LineProjSequenceSegmentMetric(uShift, uVar, minU, maxU, Math.sqrt(sumComp*sumCompMean), countPairs, listExpectedProjsOrdered.size());
			LineProjSequenceSegmentMetric metric = new LineProjSequenceSegmentMetric(uShift, uVar, minU, maxU, sumCompMean, countPairs, listExpectedProjsOrdered.size());
			mapProjSequenceSegmentMetrics.put(metric, lstNodes_);
			//*PRINTIMGS
			log.debug("lstNodes_(metric): " + "(" + metric + "):" + lstNodes_);
			//*/
		}

		////////
		//// 2 - Adiciona ao VLineMap as associações entre as projeções (VLines associadas) medidas e linhas do mapa.
		if (mapProjSequenceSegmentMetrics.keySet().iterator().hasNext())
		{
			Object metric = mapProjSequenceSegmentMetrics.keySet().iterator().next();
			ListHead lstNodesFirst = (ListHead) mapProjSequenceSegmentMetrics.get(metric);
			//*PRINTIMGS
			log.debug("metric: " + metric);
			log.debug("lstNodesFirst(selected): " + lstNodesFirst);
			//*/

			HashSet setLinesOk = new HashSet();
			while (lstNodesFirst != null)
			{
				VLineRef lineRef = lstNodesFirst.pair.getRefProj().getLineRef();
				VLineProj lMeasProj = lstNodesFirst.pair.getMeasProj();
				VLine line = lineMap.getVLine4LastMeasuredProjection(lMeasProj);
				lineMap.setWorldMapMatchByProjection(line, lineRef);
				setLinesOk.add(line);

				if (strPairs.length() > 2)
					strPairs += ", ";
				strPairs += "(" + numFrmt.format(lMeasProj.getU()) + ", " + numFrmt.format(lstNodesFirst.pair.getRefProj().getU()) + ")";
				countCount++;

				lstNodesFirst = lstNodesFirst.prev;
			}

/*
			log.debug("newLineProjs.arrayProjMeasures.length: " + newLineProjs.arrayProjMeasures.length);
			for (int measIdx = 0; measIdx < newLineProjs.arrayProjMeasures.length; measIdx++)
			{
				VLineProj lMeasProj = newLineProjs.arrayProjMeasures[measIdx];
				VLine line = lineMap.getVLine4LastMeasuredProjection(lMeasProj);
				log.debug("lMeasProj: " + lMeasProj + ", line.mapIdx: " + line.mapIdx + ", lMeasProj.mapIdx: " + lMeasProj.mapIdx);
				if (!setLinesOk.contains(line))
				{
					log.debug("DUMPING REFS");
					lineMap.unmatchWithMap(line);
				}
			}
/*/
			if (setLinesOk.size() > 0)
			{
				ArrayList listLines = new ArrayList();
				Iterator itLine = lineMap.getMappedLineIndexIterator();
				while (itLine.hasNext())
				{
					VLine line = lineMap.nextLine(itLine);
					listLines.add(line);
				}
				itLine = listLines.iterator();
				while (itLine.hasNext())
				{
					VLine line = (VLine) itLine.next();
					//*PRINTIMGS
					log.debug("line.mapIdx: " + line.mapIdx);
					//*/
					if (!setLinesOk.contains(line))
					{
						//*PRINTIMGS
						log.debug("DUMPING REFS");
						//*/
						lineMap.unmatchWithMap(line);
					}
				}
			}
			else
			{
				lineMap.unmatchAllWithMap();
			}
//*/
		}
		logVisionMapPairsCount.debug(countCount + ";");
		logVisionMapPairs.debug(strPairs);
		
		////////
		//// 3 - No VLineMap, verifica se a Vline já foi associada a mesma linah do mapa. Se sim,e se por três vezes, a projeção
		// passa a ser usada como medição, e o esperado é calculado usando a linha do mapa e  posição do robo.

	}


	private List [] arrayListPair_MarcoOrdered = new List[100];

	private List getPossiblePaths(List listExpectedProjsOrdered, double [] arrayMatchesUShiftOrdered, int [] arrayMatchesUShiftOrderedIndex, LineProjMatchPair [] arrayMatches, int idxUShiftFirst, int idxUShiftLast, boolean forceUseOfLastNode, int stride)
	{
		// limpa limpa
		for (int i = 0; i < listExpectedProjsOrdered.size(); i++)
		{
			if (arrayListPair_MarcoOrdered[i] == null)
				arrayListPair_MarcoOrdered[i] = new ArrayList();
			else
				arrayListPair_MarcoOrdered[i].clear();
		}

		// Reordena as projeções selecionadas segundo a posição da projeção
		// esperada dos marcos.
		for (int i = idxUShiftFirst; i <= idxUShiftLast; i++)
		{
			int idxPair = arrayMatchesUShiftOrderedIndex[i];
			LineProjMatchPair pair = arrayMatches[idxPair];
			int idxMarco = idxPair/stride;
			arrayListPair_MarcoOrdered[idxMarco].add(pair);
		}

		// Cria grafo.
		IDirectedGraph dirGraph = GraphFactory.createDirectedGraph(GraphFactory.standardGraph);
		IEditableGraph editGraph = (IEditableGraph) dirGraph;
		IAccessibleGraph accGraph = (IAccessibleGraph) dirGraph;
		IGraphFuncFactory funcFact = dirGraph.getGraphFuncFactory();
		IObjectNodeGraphFunc pairToNode = funcFact.createObjectNodeGraphFunc("pair_to_node");

		// Adiciona os nós.
		List listFirst = null, listBehind = null;
		//*PRINTIMGS
		log.debug("listExpectedProjsOrdered.size():" + listExpectedProjsOrdered.size());
		//*/
		for (int i = 0; i < listExpectedProjsOrdered.size(); i++)
		{
			if (arrayListPair_MarcoOrdered[i].size() > 0)
			{
				Iterator itPairs = arrayListPair_MarcoOrdered[i].iterator();
				while (itPairs.hasNext())
				{
					LineProjMatchPair pair = (LineProjMatchPair) itPairs.next();
					VLineProj proj = pair.getMeasProj();
					INode node = editGraph.addNode();
					//*PRINTIMGS
					log.debug("nó: " + pair);
					//*/
					pair.setNode(node);
					pairToNode.setValue(node, pair);
					if (listBehind != null)
					{
						Iterator itPairsAnt = listBehind.iterator();
						while (itPairsAnt.hasNext())
						{
							LineProjMatchPair pairAnt = (LineProjMatchPair) itPairsAnt.next();
							VLineProj projAnt = pairAnt.getMeasProj();
							if (proj.getU() - projAnt.getU() > 0)
							{
								IDirectedArc arc = editGraph.addArc(pairAnt.getNode(), pair.getNode());
								//*PRINTIMGS
								log.debug("arco: " + pairAnt + " -> " + pair);
								//*/
							}
						}
					}
				}
				if (listFirst == null)
					listFirst = arrayListPair_MarcoOrdered[i];
				listBehind = arrayListPair_MarcoOrdered[i];
				//*PRINTIMGS
				log.debug("listFirst:" + listFirst);
				log.debug("listBehind:" + listBehind);
				//*/
			}
		}

		ArrayList stackHeads = new ArrayList();
		ArrayList listPaths = new ArrayList();
		Iterator itPairs = listFirst.iterator();
		while (itPairs.hasNext())
		{
			LineProjMatchPair pair = (LineProjMatchPair) itPairs.next();
			ListHead lstHead = new ListHead(pair, pair.getNode());
			stackHeads.add(lstHead);
		}
//		log.debug("stackHeads:" + stackHeads.size());
		while (stackHeads.size() > 0)
		{
			ListHead lstHead = (ListHead) stackHeads.remove(stackHeads.size() - 1);
//			log.debug("stackHeads(1):" + stackHeads.size());
			INodeIterator nodeIt = accGraph.getNextNodes(lstHead.node);
//			log.debug("pair from:" + (LineProjMatchPair) pairToNode.getValue(lstHead.node));
			boolean bHasNexts = false;
			while (nodeIt.hasNext())
			{
				INode nodeNext = nodeIt.nextNode();
//				log.debug("pair next:" + (LineProjMatchPair) pairToNode.getValue(nodeNext));
				ListHead lstNext = new ListHead(lstHead, (LineProjMatchPair) pairToNode.getValue(nodeNext), nodeNext);
				stackHeads.add(lstNext);
				bHasNexts = true;
			}
			if (!bHasNexts)
				listPaths.add(lstHead);
//			log.debug("stackHeads(2):" + stackHeads.size());
		}
		return listPaths;
	}
}


class ListHead
{
	private static NumberFormat numFrmt = NumberFormat.getInstance();
	static
	{
		numFrmt.setMinimumFractionDigits(2);
		numFrmt.setMaximumFractionDigits(2);
		numFrmt.setGroupingUsed(false);
	}

	
	public LineProjMatchPair pair = null;
	public INode node = null;
	public ListHead prev = null;

	
	public ListHead(LineProjMatchPair pair, INode node)
	{
		this.pair = pair;
		this.node = node;
		this.prev = null;
	}


	public ListHead(ListHead prev, LineProjMatchPair pair, INode node)
	{
		this.pair = pair;
		this.node = node;
		this.prev = prev;
	}


	public String toString()
	{
		StringBuffer sb = new StringBuffer();
		sb.append("[");
		ListHead lstNodes = this;
		while (lstNodes != null)
		{
			LineProjMatchPair pair = lstNodes.pair;

			sb.append("|");
			sb.append(numFrmt.format(pair.getUShift()));
			sb.append("|, ");
			sb.append(numFrmt.format(pair.getRefProj().getU()));
			sb.append(", ");
			sb.append(numFrmt.format(pair.getMeasProj().getU()));
			lstNodes = lstNodes.prev;
			if (lstNodes != null)
				sb.append(" --> ");
		}
		sb.append("]");
		return sb.toString();
	}
}

