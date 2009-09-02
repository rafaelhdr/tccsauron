package br.com.r4j.research.pose2destimation;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.DoubleSquareMatrix;
import br.com.r4j.robosim.Pose2D;


/** 
 * Classe centralizadora dos dados, reponsável por gerar as estimativas.
 *
 */
public class TestPose2DGrabber implements Pose2DGrabber
{
	private Pose2D poseEstimateTotalOld = null;

	
	public TestPose2DGrabber()
	{
		poseEstimateTotalOld = new Pose2D(0, 0, 0);
	}

	
	public Pose2D getPose(long imageTimestamp)
	{
		poseEstimateTotalOld = new Pose2D(poseEstimateTotalOld.getX(), poseEstimateTotalOld.getY(), poseEstimateTotalOld.getTheta());
		return poseEstimateTotalOld;

	}


	public Pose2D getMovement(long imageTimestamp)
	{
		return new Pose2D(0, 0, 0);
	}


	public AbstractDoubleSquareMatrix getPoseCovar(long imageTimestamp)
	{
		return null;
	}


	public AbstractDoubleSquareMatrix getMovementCovar(long imageTimestamp)
	{
		AbstractDoubleSquareMatrix covarInc = new DoubleSquareMatrix(3);
		covarInc.setElement(0, 0, 30);
		covarInc.setElement(1, 1, 30);
		covarInc.setElement(2, 2, 20*Math.PI/180);
		return covarInc;
	}
}
