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


/**
 *
 */
public interface LineProjPairGenerator
{
	public PairedLines createLineProjPairs(RigidBodyTranformation2D cameraTrafo, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar, LineExtrationResult newLineProjs, LineMatchingResult lineMatchingResult, AbstractDoubleSquareMatrix cameraPoseCovar, CameraModel camModel, VLineMap lineMap, WorldMap worldMap, DifferenceSimpleMatchingPhaseMatcher callback);

	public void setUseWindow(boolean bVal);
	public void setUseDirectionWindow(boolean bVal);
	public void setDirectWindowSize(int val);
	public void setHashVectorTreshold(double hashVectorTreshold);
}


