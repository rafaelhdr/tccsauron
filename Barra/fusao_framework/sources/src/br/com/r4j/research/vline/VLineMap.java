package br.com.r4j.research.vline;

import java.awt.Color;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.TreeMap;
import java.util.TreeSet;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.robosim.Pose2D;



/**
 * Class que representa a estrutura de linhas verticais 3D para proje��es
 * 2D encontradas.
 *
 * O mapeamento leva em conta medi��es e proje��es. Tamb�m indica se uma medi��o
 * est� visivel e se uma proje��o est� visivel.
 *
 * O armazenamento ta uma zona:
 *  - basicamente, est� tudo em array, mas o acesso � por um iterator de �ndices que
 * s�o usados para acessar os arrays. Os �ndices por sua vez est�o em LinkedList e HashSets.
 *
 *  - tem um armazenamento em Lists para os visible e non-visible, por�m n�o est�o sendo usados.
 *    Talvez sej� interessante trocar isso por um que mantenha stacks dos �ndices para poder
 *    fazer esses iterators ...
 *
 *  - um m�todo que gera a cada itera��o a lista de Lines ordendas por u.
 *
 *
 */
public class VLineMap
{
	private static Log log = LogFactory.getLog(VLineMap.class.getName());

	public static int NO_BAND = -1;
	public static int FIRST_BAND = 0;
	public static int SECOND_BAND = 1;
	public static int BOTH_BANDS = 2;


	private int INITIAL_INCREMENTS = 2;
	private int INCREMETAL_SIZE = 100;

	// �ndices livres.
	private LinkedList stackFreeIndexes = null;
	private TreeSet stackUsedIndexes = null;
	private TreeSet stackStateIndexes = null;
	private TreeSet stackMappedIndexes = null;

	private int lineVisCount = 0;


	private Color [] arrayDebugColor = null;
	public int nextColorIdx = 0;
	public int [] arrayTTL = null;


	private VLine [] arrayVLine = null;
	private VLine [] arrayVLinePrelim = null;


	private boolean [] arrayIsLineState = null;
	private VLine [] arrayLineModel = null;
	private int [] arrayLineModelProjCount = null;
	private AbstractDoubleSquareMatrix [] arrayLineModelCovar = null;
	private RigidBodyTranformation2D [] arrayLineModelRigidBodyTranformation2D = null;

	private VLineProj [] arrayVLineBeforeLastMeasuredProj = null;
	private RigidBodyTranformation2D [] arrayRigidTrafoBeforeLastMeasuredProj = null;


	// Indica se foi encontrado uma medi��o que possa ser atribuida a linha na �tlima itera��o.
	private boolean [] arrayVisible = null;

	// Indica se a estimativa da linha 3D pode ser considerada v�lida.
	private boolean [] arrayHasPrelimModel = null;
	private boolean [] arrayHasLineModel = null;
	private boolean [] arrayVLineHasIdleMeasure = null;

	private VLineRef [] arrayMapLink = null;


	// Indica se o �ltimo match foi feito atrav�s de uma ou das duas bandas.
	private int [] arrayBand = null;

	// Primeira proje��o medida associada com a linha.
	private VLineProj [] arrayVLineFirstMeasuredProj = null;
	private VLineProj [] arrayVLineSecondMeasuredProj = null;
	// �ltima proje��o medida associada com a linha.
	private VLineProj [] arrayVLineMeasuredProj = null;

	private VLineRef [] arrayVLineAssocVLineRef = null;
	public int [] arrayVLineAssocVLineRefCount = null;
	
	private Pose2D [] arrayVLineFirstMeasuredProjPoseCam = null;
	private Pose2D [] arrayVLineSecondMeasuredProjPoseCam = null;
	private Pose2D [] arrayVLineLastMeasuredProjPoseCam = null;

	private AbstractDoubleSquareMatrix [] arrayVLineFirstMeasuredProjPoseCamCovar = null;
	private AbstractDoubleSquareMatrix [] arrayVLineSecondMeasuredProjPoseCamCovar = null;
	private AbstractDoubleSquareMatrix [] arrayVLineLastMeasuredProjPoseCamCovar = null;

	private RigidBodyTranformation2D [] arrayRigidTrafoFirstMeasuredProj = null;
	private RigidBodyTranformation2D [] arrayRigidTrafoSecondMeasuredProj = null;
	private RigidBodyTranformation2D [] arrayRigidTrafoLastMeasuredProj = null;



//	private CameraModel camModel = null;
//	private ProjectionModel projModel = null;


/*
	public CameraModel getCameraModel()	{return this.camModel;}
	public void setCameraModel(CameraModel camModel)	{this.camModel = camModel;}
	public ProjectionModel getProjectionModel()	{return this.projModel;}
	public void setProjectionModel(ProjectionModel projModel)	{this.projModel = projModel;}
//*/

	public VLineMap()
	{
		stackFreeIndexes = new LinkedList();
		stackUsedIndexes = new TreeSet();
		stackStateIndexes = new TreeSet();
		stackMappedIndexes = new TreeSet();

		arrayIsLineState = new boolean[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayLineModel = new VLine[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayLineModelProjCount = new int[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayLineModelCovar = new AbstractDoubleSquareMatrix[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayLineModelRigidBodyTranformation2D = new RigidBodyTranformation2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineBeforeLastMeasuredProj = new VLineProj[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayRigidTrafoBeforeLastMeasuredProj = new RigidBodyTranformation2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];

		arrayVLine = new VLine[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVisible = new boolean[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayHasLineModel = new boolean[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineHasIdleMeasure = new boolean[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayBand = new int[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineMeasuredProj = new VLineProj[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineAssocVLineRef = new VLineRef[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineAssocVLineRefCount = new int[INITIAL_INCREMENTS*INCREMETAL_SIZE];

		arrayVLineFirstMeasuredProj = new VLineProj[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineFirstMeasuredProjPoseCam = new Pose2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayRigidTrafoFirstMeasuredProj = new RigidBodyTranformation2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];

		arrayVLineFirstMeasuredProjPoseCamCovar = new AbstractDoubleSquareMatrix [INITIAL_INCREMENTS*INCREMETAL_SIZE];;
		arrayVLineSecondMeasuredProjPoseCamCovar = new AbstractDoubleSquareMatrix [INITIAL_INCREMENTS*INCREMETAL_SIZE];;
		arrayVLineLastMeasuredProjPoseCamCovar = new AbstractDoubleSquareMatrix [INITIAL_INCREMENTS*INCREMETAL_SIZE];;

		arrayVLinePrelim = new VLine[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayHasPrelimModel = new boolean[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayMapLink = new VLineRef[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineSecondMeasuredProj = new VLineProj[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineSecondMeasuredProjPoseCam = new Pose2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayVLineLastMeasuredProjPoseCam = new Pose2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayRigidTrafoSecondMeasuredProj = new RigidBodyTranformation2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];
		arrayRigidTrafoLastMeasuredProj = new RigidBodyTranformation2D[INITIAL_INCREMENTS*INCREMETAL_SIZE];

		arrayDebugColor = new Color[this.getCurrentStorageSize()];
		arrayTTL = new int[this.getCurrentStorageSize()];

		for (int i = 0; i < INITIAL_INCREMENTS*INCREMETAL_SIZE; i++)
		{
			Integer intg = new Integer(i);
			stackFreeIndexes.add(intg);
		}
	}

	
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// Auxiliares para sub-classe

	protected int getCurrentStorageSize()
	{
		return arrayVLine.length;
	}


/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// M�todos de inser��o e remo��o

	/**
	 * Deve ser usado pelas sub-classes para aumentar o tamanho dos arrays internos.
	 * Primeiro chama o super-m�todo, depois faz o qeu deve.
	 */
	protected void grow()
	{
		boolean [] arrayIsLineStateTmp = new boolean[arrayVLine.length + INCREMETAL_SIZE];
		VLine [] arrayLineModelTmp = new VLine[arrayVLine.length + INCREMETAL_SIZE];
		int [] arrayLineModelProjCountTmp = new int[arrayVLine.length + INCREMETAL_SIZE];
		AbstractDoubleSquareMatrix [] arrayLineModelCovarTmp = new AbstractDoubleSquareMatrix[arrayVLine.length + INCREMETAL_SIZE];
		RigidBodyTranformation2D [] arrayLineModelRigidBodyTranformation2DTmp = new RigidBodyTranformation2D[arrayVLine.length + INCREMETAL_SIZE];
		VLineProj [] arrayVLineBeforeLastMeasuredProjTmp = new VLineProj[arrayVLine.length + INCREMETAL_SIZE];
		RigidBodyTranformation2D [] arrayRigidTrafoBeforeLastMeasuredProjTmp = new RigidBodyTranformation2D[arrayVLine.length + INCREMETAL_SIZE];

		VLine [] arrayVLineTmp = new VLine[arrayVLine.length + INCREMETAL_SIZE];
		boolean [] arrayVisibleTmp = new boolean[arrayVLine.length + INCREMETAL_SIZE];
		boolean [] arrayHasLineModelTmp = new boolean[arrayVLine.length + INCREMETAL_SIZE];
		boolean [] arrayVLineHasIdleMeasureTmp = new boolean[arrayVLine.length + INCREMETAL_SIZE];
		int [] arrayBandTmp = new int[arrayVLine.length + INCREMETAL_SIZE];
		VLineProj [] arrayVLineMeasuredProjTmp = new VLineProj[arrayVLine.length + INCREMETAL_SIZE];
		VLineProj [] arrayVLineFirstMeasuredProjTmp = new VLineProj[arrayVLine.length + INCREMETAL_SIZE];
		Pose2D [] arrayVLineFirstMeasuredProjPoseCamTmp = new Pose2D[arrayVLine.length + INCREMETAL_SIZE];
		RigidBodyTranformation2D [] arrayRigidTrafoFirstMeasuredProjTmp = new RigidBodyTranformation2D[arrayVLine.length + INCREMETAL_SIZE];

		AbstractDoubleSquareMatrix [] arrayVLineFirstMeasuredProjPoseCamCovarTmp = new AbstractDoubleSquareMatrix[arrayVLine.length + INCREMETAL_SIZE];
		AbstractDoubleSquareMatrix [] arrayVLineSecondMeasuredProjPoseCamCovarTmp = new AbstractDoubleSquareMatrix[arrayVLine.length + INCREMETAL_SIZE];
		AbstractDoubleSquareMatrix [] arrayVLineLastMeasuredProjPoseCamCovarTmp = new AbstractDoubleSquareMatrix[arrayVLine.length + INCREMETAL_SIZE];


		VLine [] arrayVLinePrelimTmp = new VLine[arrayVLine.length + INCREMETAL_SIZE];
		boolean [] arrayHasPrelimModelTmp = new boolean[arrayVLine.length + INCREMETAL_SIZE];
		VLineRef [] arrayMapLinkTmp = new VLineRef[arrayVLine.length + INCREMETAL_SIZE];
		VLineProj [] arrayVLineSecondMeasuredProjTmp = new VLineProj[arrayVLine.length + INCREMETAL_SIZE];
		Pose2D [] arrayVLineSecondMeasuredProjPoseCamTmp = new Pose2D[arrayVLine.length + INCREMETAL_SIZE];
		Pose2D [] arrayVLineLastMeasuredProjPoseCamTmp = new Pose2D[arrayVLine.length + INCREMETAL_SIZE];
		RigidBodyTranformation2D [] arrayRigidTrafoSecondMeasuredProjTmp = new RigidBodyTranformation2D[arrayVLine.length + INCREMETAL_SIZE];
		RigidBodyTranformation2D [] arrayRigidTrafoLastMeasuredProjTmp = new RigidBodyTranformation2D[arrayVLine.length + INCREMETAL_SIZE];
		VLineRef [] arrayVLineAssocVLineRefTmp = new VLineRef[arrayVLine.length + INCREMETAL_SIZE];
		int [] arrayVLineAssocVLineRefCountTmp = new int[arrayVLine.length + INCREMETAL_SIZE];

		for (int i = 0; i < arrayVLine.length; i++)
		{
			arrayIsLineStateTmp[i] = arrayIsLineState[i];
			arrayLineModelTmp[i] = arrayLineModel[i];
			arrayLineModelProjCountTmp[i] = arrayLineModelProjCount[i];
			arrayLineModelCovarTmp[i] = arrayLineModelCovar[i];
			arrayLineModelRigidBodyTranformation2DTmp[i] = arrayLineModelRigidBodyTranformation2D[i];
			arrayVLineBeforeLastMeasuredProjTmp[i] = arrayVLineBeforeLastMeasuredProj[i];
			arrayRigidTrafoBeforeLastMeasuredProjTmp[i] = arrayRigidTrafoBeforeLastMeasuredProj[i];

			arrayVLineTmp[i] = arrayVLine[i];
			arrayVisibleTmp[i] = arrayVisible[i];
			arrayHasLineModelTmp[i] = arrayHasLineModel[i];
			arrayVLineHasIdleMeasureTmp[i] = arrayVLineHasIdleMeasure[i];
			arrayBandTmp[i] = arrayBand[i];
			arrayVLineMeasuredProjTmp[i] = arrayVLineMeasuredProj[i];
			arrayVLineAssocVLineRefTmp[i] = arrayVLineAssocVLineRef[i];
			arrayVLineAssocVLineRefCountTmp[i] = arrayVLineAssocVLineRefCount[i];

			arrayVLineFirstMeasuredProjTmp[i] = arrayVLineFirstMeasuredProj[i];
			arrayVLineFirstMeasuredProjPoseCamTmp[i] = arrayVLineFirstMeasuredProjPoseCam[i];
			arrayRigidTrafoFirstMeasuredProjTmp[i] = arrayRigidTrafoFirstMeasuredProj[i];

			arrayVLineFirstMeasuredProjPoseCamCovarTmp[i] = arrayVLineFirstMeasuredProjPoseCamCovar[i];
			arrayVLineSecondMeasuredProjPoseCamCovarTmp[i] = arrayVLineSecondMeasuredProjPoseCamCovar[i];
			arrayVLineLastMeasuredProjPoseCamCovarTmp[i] = arrayVLineLastMeasuredProjPoseCamCovar[i];

			arrayVLinePrelimTmp[i] = arrayVLinePrelim[i];
			arrayHasPrelimModelTmp[i] = arrayHasPrelimModel[i];
			arrayMapLinkTmp[i] = arrayMapLink[i];
			arrayVLineSecondMeasuredProjTmp[i] = arrayVLineSecondMeasuredProj[i];
			arrayVLineSecondMeasuredProjPoseCamTmp[i] = arrayVLineSecondMeasuredProjPoseCam[i];
			arrayVLineLastMeasuredProjPoseCamTmp[i] = arrayVLineLastMeasuredProjPoseCam[i];
			arrayRigidTrafoSecondMeasuredProjTmp[i] = arrayRigidTrafoSecondMeasuredProj[i];
			arrayRigidTrafoLastMeasuredProjTmp[i] = arrayRigidTrafoLastMeasuredProj[i];
		}

		for (int i = arrayVLine.length; i < arrayVLineTmp.length; i++)
		{
			Integer intg = new Integer(i);
			stackFreeIndexes.add(intg);
		}

		arrayIsLineState = arrayIsLineStateTmp;
		arrayLineModel = arrayLineModelTmp;
		arrayLineModelProjCount = arrayLineModelProjCountTmp;
		arrayLineModelCovar = arrayLineModelCovarTmp;
		arrayLineModelRigidBodyTranformation2D = arrayLineModelRigidBodyTranformation2DTmp;
		arrayVLineBeforeLastMeasuredProj = arrayVLineBeforeLastMeasuredProjTmp;
		arrayRigidTrafoBeforeLastMeasuredProj = arrayRigidTrafoBeforeLastMeasuredProjTmp;

		arrayVLine = arrayVLineTmp;
		arrayVisible = arrayVisibleTmp;
		arrayHasLineModel = arrayHasLineModelTmp;
		arrayVLineHasIdleMeasure = arrayVLineHasIdleMeasureTmp;
		arrayBand = arrayBandTmp;
		arrayVLineMeasuredProj = arrayVLineMeasuredProjTmp;
		arrayVLineAssocVLineRef = arrayVLineAssocVLineRefTmp;
		arrayVLineAssocVLineRefCount = arrayVLineAssocVLineRefCountTmp;

		arrayVLineFirstMeasuredProj = arrayVLineFirstMeasuredProjTmp;
		arrayVLineFirstMeasuredProjPoseCam = arrayVLineFirstMeasuredProjPoseCamTmp;
		arrayRigidTrafoFirstMeasuredProj = arrayRigidTrafoFirstMeasuredProjTmp;

		arrayVLineFirstMeasuredProjPoseCamCovar = arrayVLineFirstMeasuredProjPoseCamCovarTmp;
		arrayVLineSecondMeasuredProjPoseCamCovar = arrayVLineSecondMeasuredProjPoseCamCovarTmp;
		arrayVLineLastMeasuredProjPoseCamCovar = arrayVLineLastMeasuredProjPoseCamCovarTmp;

		arrayVLinePrelim = arrayVLinePrelimTmp;
		arrayHasPrelimModel = arrayHasPrelimModelTmp;
		arrayMapLink = arrayMapLinkTmp;
		arrayVLineSecondMeasuredProj = arrayVLineSecondMeasuredProjTmp;
		arrayVLineSecondMeasuredProjPoseCam = arrayVLineSecondMeasuredProjPoseCamTmp;
		arrayVLineLastMeasuredProjPoseCam = arrayVLineLastMeasuredProjPoseCamTmp;
		arrayRigidTrafoSecondMeasuredProj = arrayRigidTrafoSecondMeasuredProjTmp;
		arrayRigidTrafoLastMeasuredProj = arrayRigidTrafoLastMeasuredProjTmp;
	
		Color [] arrayDebugColorTmp = new Color[this.getCurrentStorageSize()];
		int [] arrayTTLTmp = new int[this.getCurrentStorageSize()];

		for (int i = 0; i < arrayDebugColor.length; i++)
		{
			arrayDebugColorTmp[i] = arrayDebugColor[i];
			arrayTTLTmp[i] = arrayTTL[i];
		}

		arrayDebugColor = arrayDebugColorTmp;
		arrayTTL = arrayTTLTmp;
	}


	/**
	 * Adiciona reta com aproxima��o grosseira do que pode sera linha.
	 *
	 */
	public int add(VLine line, VLineProj projFirstProj, Pose2D firstMeasuredProjPoseCam, AbstractDoubleSquareMatrix firstMeasuredProjPoseCamCovar, RigidBodyTranformation2D rigidTrafoFirstMeasuredProj)
	{
		if (stackFreeIndexes.size() == 0)
			this.grow();

		Integer intIdx = intIdx = (Integer) stackFreeIndexes.remove(stackFreeIndexes.size() - 1);
		stackUsedIndexes.add(intIdx);
		int idx = intIdx.intValue();

		arrayVLine[idx] = line;
		arrayBand[idx] = NO_BAND;
		arrayVisible[idx] = false;
		line.mapIdx = idx;
		line.mapPrevStateVectIdx = -1;
		line.mapCurrStateVectIdx = -1;

		arrayRigidTrafoFirstMeasuredProj[line.mapIdx] = rigidTrafoFirstMeasuredProj;
		arrayVLineFirstMeasuredProj[line.mapIdx] = projFirstProj;

//		arrayVLineBeforeLastMeasuredProj[line.mapIdx] = arrayVLineMeasuredProj[line.mapIdx];
		arrayVLineMeasuredProj[line.mapIdx] = projFirstProj;
		projFirstProj.mapIdx = line.mapIdx;

		arrayVLineAssocVLineRef[line.mapIdx] = null;
		arrayVLineAssocVLineRefCount[line.mapIdx] = 0;

		arrayHasLineModel[line.mapIdx] = false;
		arrayVLineHasIdleMeasure[line.mapIdx] = false;
		arrayIsLineState[line.mapIdx] = false;

		return idx;
	}


	public void setFirstMeasuredProjection(VLine line, VLineProj p1, RigidBodyTranformation2D poseFirst)
	{
		arrayRigidTrafoFirstMeasuredProj[line.mapIdx] = poseFirst;
		p1.mapIdx = line.mapIdx;
		arrayVLineFirstMeasuredProj[line.mapIdx] = p1;

		arrayHasLineModel[line.mapIdx] = false;
	}


	public void setLineModel(VLine line, VLine lineBase, int countProjsUsed, AbstractDoubleSquareMatrix moveCovar)
	{
		arrayLineModel[line.mapIdx] = lineBase;
		lineBase.mapIdx = line.mapIdx;
		arrayLineModelProjCount[line.mapIdx] = countProjsUsed;
		arrayLineModelCovar[line.mapIdx] = moveCovar;
		if (arrayIsLineState[line.mapIdx])
		{
			log.debug("algo errado: line: " + line);
			log.debug("algo errado: lineBase: " + lineBase);

		}

		arrayHasLineModel[line.mapIdx] = true;
		arrayVLineHasIdleMeasure[line.mapIdx] = false;
		arrayIsLineState[line.mapIdx] = false;
	}


	public void invalidateModel(VLine line)
	{
		arrayLineModelProjCount[line.mapIdx] = 2;
		arrayHasLineModel[line.mapIdx] = false;
		arrayVLineHasIdleMeasure[line.mapIdx] = false;
		arrayIsLineState[line.mapIdx] = false;
	}


	public void setLineState(VLine line, VLine lineBase, AbstractDoubleSquareMatrix moveCovar)
	{
		arrayLineModel[line.mapIdx] = lineBase;
		lineBase.mapIdx = line.mapIdx;
		arrayLineModelCovar[line.mapIdx] = moveCovar;
		if (!arrayIsLineState[line.mapIdx])
		{
			log.debug("algo errado(1): line: " + line);
			log.debug("algo errado(1): lineBase: " + lineBase);
		}
	}

	
	public void setLineAsState(VLine line, VLine lineBase, AbstractDoubleSquareMatrix iniCovar, RigidBodyTranformation2D cameraTrafo)
	{
		arrayLineModel[line.mapIdx] = lineBase;
		lineBase.mapIdx = line.mapIdx;
		lineBase.setCovar(iniCovar);
//		arrayLineModelCovar[line.mapIdx] = iniCovar;
		arrayLineModelRigidBodyTranformation2D[line.mapIdx] = cameraTrafo;
	
		if (arrayHasLineModel[line.mapIdx] != true)
			log.error("ERRO(1): " + (arrayHasLineModel[line.mapIdx] != true));

		arrayIsLineState[line.mapIdx] = true;
		arrayVLineHasIdleMeasure[line.mapIdx] = false;
	}

	
	public void setMeasuredProjection(VLine line, VLineProj projMeasured, Pose2D camPose, AbstractDoubleSquareMatrix cameraPoseCovar, RigidBodyTranformation2D cameraTrafo)
	{
		log.debug("setMeasuredProjection:arrayVLineHasIdleMeasure[line.mapIdx]: " + arrayVLineHasIdleMeasure[line.mapIdx] + ", isLineState(line): " + isLineState(line));
		if (!arrayVLineHasIdleMeasure[line.mapIdx] && isLineState(line))
		{
			stackStateIndexes.add(new Integer(line.mapIdx));
		}


//		arrayVLineBeforeLastMeasuredProj[line.mapIdx] = arrayVLineMeasuredProj[line.mapIdx];
		arrayVLineMeasuredProj[line.mapIdx] = projMeasured; 
		projMeasured.mapIdx = line.mapIdx;

		arrayVLineHasIdleMeasure[line.mapIdx] = true;
	}


	public void setWorldMapMatchByProjection(VLine line, VLineRef lineRef)
	{
		if (arrayVLineAssocVLineRef[line.mapIdx] == lineRef)
			arrayVLineAssocVLineRefCount[line.mapIdx]++;
		else
		{
			if (stackMappedIndexes.contains(new Integer(line.mapIdx)))
				stackMappedIndexes.remove(new Integer(line.mapIdx));

			arrayVLineAssocVLineRef[line.mapIdx] = lineRef;
			arrayVLineAssocVLineRefCount[line.mapIdx] = 1;
		}

		log.debug("setWorldMapMatchByProjection: " + arrayVLineAssocVLineRefCount[line.mapIdx]);
		if (arrayVLineAssocVLineRefCount[line.mapIdx] > 1)
		{
			log.debug("setWorldMapMatchByProjection:aceito!");
			stackMappedIndexes.add(new Integer(line.mapIdx));
			stackStateIndexes.remove(new Integer(line.mapIdx));
		}
	}


	public void unmatchWithMap(VLine line)
	{
		arrayVLineAssocVLineRefCount[line.mapIdx] = 0;
		if (stackMappedIndexes.contains(new Integer(line.mapIdx)))
			stackMappedIndexes.remove(new Integer(line.mapIdx));
	}


	public void unmatchAllWithMap()
	{
		Iterator ititit = stackMappedIndexes.iterator();
		while (ititit.hasNext())
		{
			Integer intStack = (Integer) ititit.next();
			arrayVLineAssocVLineRefCount[intStack.intValue()] = 0;
		}
		stackMappedIndexes.clear();
	}


	public void remove(VLine line)
	{
		log.debug("removing ... : " + line);

		Integer intg = new Integer(line.mapIdx);

		stackFreeIndexes.add(intg);
		stackUsedIndexes.remove(intg);
		stackStateIndexes.remove(intg);
		stackMappedIndexes.remove(intg);

		arrayRigidTrafoFirstMeasuredProj[line.mapIdx] = null;
		arrayVLineFirstMeasuredProj[line.mapIdx] = null;

		arrayVLineBeforeLastMeasuredProj[line.mapIdx] = null;
		arrayVLineMeasuredProj[line.mapIdx] = null;

		arrayVLineAssocVLineRef[line.mapIdx] = null;
		arrayVLineAssocVLineRefCount[line.mapIdx] = -1;

		arrayHasLineModel[line.mapIdx] = false;
		arrayVLineHasIdleMeasure[line.mapIdx] = false;
		arrayIsLineState[line.mapIdx] = false;
	}


/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// M�todos de itera��o de baixo custo

	public int getNumberOfLines()	{return stackStateIndexes.size();}
	public int getNumberOfStateLines()	{return stackStateIndexes.size();}
	public int getNumberOfMappedLines()	{return stackMappedIndexes.size();}


	public Iterator getLineIndexIterator()	{return stackUsedIndexes.iterator();}
	public Iterator getStateLineIndexIterator()	{return stackStateIndexes.iterator();}
	public Iterator getMappedLineIndexIterator()	{return stackMappedIndexes.iterator();}

	public boolean hasNext(Iterator itLines)	{return itLines.hasNext();}

	public VLine nextLine(Iterator itLines)
	{
		Integer intg = (Integer) itLines.next();
		return arrayVLine[intg.intValue()];
	}


	public VLine nextLineModel(Iterator itLines)
	{
		Integer intg = (Integer) itLines.next();
		return arrayLineModel[intg.intValue()];
	}


/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// M�todos para acessar atributos

	public boolean isLineMapMeasure(VLine line)	{return arrayVLineAssocVLineRefCount[line.mapIdx] > 2;}
	public boolean isLineState(VLine line)	{return arrayIsLineState[line.mapIdx];}
	public boolean hasLineModel(VLine line)	{return arrayHasLineModel[line.mapIdx];}


	public boolean hasTwoMeasures(VLine line)
	{
		return arrayVLineMeasuredProj[line.mapIdx] != null && arrayVLineBeforeLastMeasuredProj[line.mapIdx] != null;
	}

				
	public VLine getLineModel(VLine line)
		{return arrayLineModel[line.mapIdx];}
	public int getLineModelProjCount(VLine line)
		{return arrayLineModelProjCount[line.mapIdx];}
	public AbstractDoubleSquareMatrix getLineModelMoveCovar(VLine line)
		{return arrayLineModelCovar[line.mapIdx];}
	public RigidBodyTranformation2D getLineModelRigidBodyTranformation2D(VLine line)
		{return arrayLineModelRigidBodyTranformation2D[line.mapIdx];}
	public void setLineModelRigidBodyTranformation2D(VLine line, RigidBodyTranformation2D trafo)
		{arrayLineModelRigidBodyTranformation2D[line.mapIdx] = trafo;}

	public RigidBodyTranformation2D getFirstMeasuredRigidBodyTranformation2D(VLine line)	
		{return arrayRigidTrafoFirstMeasuredProj[line.mapIdx];}
	public VLineProj getFirstMeasuredProjection(VLine line)	
		{return arrayVLineFirstMeasuredProj[line.mapIdx];}

	public VLineProj getBeforeLastMeasuredProjection(VLine line)	
		{return arrayVLineBeforeLastMeasuredProj[line.mapIdx];}
	public void setBeforeLastMeasuredProjection(VLine line, VLineProj proj)	
		{arrayVLineBeforeLastMeasuredProj[line.mapIdx] = proj;}
	public RigidBodyTranformation2D getBeforeLastMeasuredRigidBodyTranformation2D(VLine line)	
		{return arrayRigidTrafoBeforeLastMeasuredProj[line.mapIdx];}
	public void setBeforeLastMeasuredRigidBodyTranformation2D(VLine line, RigidBodyTranformation2D cameraTrafo)
		{arrayRigidTrafoBeforeLastMeasuredProj[line.mapIdx] = cameraTrafo;}

	public VLineProj getLastMeasuredProjection(VLine line)	
		{return arrayVLineMeasuredProj[line.mapIdx];}
	public RigidBodyTranformation2D getLastMeasuredRigidBodyTranformation2D(VLine line)	
		{return arrayRigidTrafoLastMeasuredProj[line.mapIdx];}
	
	public int getMatchedBand(VLine line)	{return arrayBand[line.mapIdx];}
	public void setMatchedBand(VLine line, int val)	{arrayBand[line.mapIdx] = val;}

	public VLine getVLine4LastMeasuredProjection(VLineProj lineP)	{return arrayVLine[lineP.mapIdx];}

	public VLineRef getAssocVLineRef(VLine line)	{return arrayVLineAssocVLineRef[line.mapIdx];}


/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// M�todos para gerar vis�es e calcular valores


	/**
	 * Constr�i em tem n*log(n) um mapa ordenado pela coordenada u da proje��o medida associada.
	 * Apenas para entradas vis�veis. V�lidas e inv�lidas.
	 *
	 *
	 */
	public TreeMap buildLastMeasuredProjectionOrderedMap()
	{
		TreeMap map = new TreeMap();
		Iterator it = this.getLineIndexIterator();
		while (this.hasNext(it))
		{
			VLine line = this.nextLine(it);
			Double uCoord = new Double(this.getLastMeasuredProjection(line).getU());
			map.put(uCoord, line);
		}
		return map;
	}

	public Color getDebugColor(VLine line)	{return arrayDebugColor[line.mapIdx];}
	public void setDebugColor(VLine line, Color clr)	{arrayDebugColor[line.mapIdx] = clr;}

	/**
 * N�o � mais usado!
 * N�o � mais usado!
 * N�o � mais usado!
 * N�o � mais usado!
 *
 *
	 * Constr�i em tem n*log(n) um mapa ordenado pela coordenada u da proje��o calculada associada.
	 * Apenas para entradas vis�veis. V�lidas e inv�lidas.
	 */
/*
	public TreeMap buildEstimatedProjectionOrderedMap()
	{
		int uPixelCount = camModel.getUAxisPixelCount();
		TreeMap map = new TreeMap();
		Iterator it = this.getLineIndexIterator();
		while (this.hasNext(it))
		{
			VLine line = this.nextLine(it);
			VLineProj lineProj = this.getEstimatedProjection(line);
			int u  = (int) lineProj.getU();
			int uSigmaError = (int) lineProj.getUErrorSigma();
			if (u + uSigmaError >= 0 || u - uSigmaError <= uPixelCount)
				map.put(new Double(u), line);
		}
		return map;
	}
//*/
}

