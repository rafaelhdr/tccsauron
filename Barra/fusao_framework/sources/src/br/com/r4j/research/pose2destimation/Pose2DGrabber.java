package br.com.r4j.research.pose2destimation;

import JSci.maths.AbstractDoubleSquareMatrix;
import br.com.r4j.robosim.Pose2D;


/** 
 * Classe centralizadora dos dados, reponsável por gerar as estimativas.
 *
 */
public interface Pose2DGrabber
{
	public Pose2D getPose(long imageTimestamp);
	public AbstractDoubleSquareMatrix getPoseCovar(long imageTimestamp);
	public Pose2D getMovement(long imageTimestamp);
	public AbstractDoubleSquareMatrix getMovementCovar(long imageTimestamp);

}
