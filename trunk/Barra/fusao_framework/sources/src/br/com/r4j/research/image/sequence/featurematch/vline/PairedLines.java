package br.com.r4j.research.image.sequence.featurematch.vline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeMap;

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




public class PairedLines
{
	public boolean [] arrayProjLastMeasuresAlreadyUsed = null;
	public boolean [] arrayProjMeasuresAlreadyUsed = null;
	public VLine [] arrayProjLastMeasuresLines = null;
	public TreeMap [] arrayTreeMapLinks = null;
	public TreeMap treeMapLinks = new TreeMap();
//	public int [] arrayPotentialMatch = null;
}


