
package br.com.r4j.research;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleSquareMatrix;
import JSci.maths.DoubleVector;
import br.com.r4j.commons.util.MatrixUtil;
import br.com.r4j.research.vline.*;
import br.com.r4j.robosim.Pose2D;


/**
 *
 */
public class RigidBodyTranformation2D
{
	private static Log log = LogFactory.getLog(RigidBodyTranformation2D.class.getName());

	private AbstractDoubleSquareMatrix rot = null;
	private AbstractDoubleSquareMatrix rotT = null;
	private AbstractDoubleVector trans = null;
	private Pose2D pose2d = null;

//*
	public RigidBodyTranformation2D(AbstractDoubleVector vectpose2d)
	{
		this(new Pose2D(vectpose2d));
	}
//*/

	public RigidBodyTranformation2D(Pose2D pose2d)
	{
		this.pose2d = pose2d;
		this.trans = new DoubleVector(2);
		trans.setComponent(0, pose2d.getX()); trans.setComponent(1, pose2d.getY());

		this.rot = new DoubleSquareMatrix(2);

		rot.setElement(0, 0, pose2d.getCosTheta()); rot.setElement(0, 1, -pose2d.getSinTheta());
		rot.setElement(1, 0, pose2d.getSinTheta()); rot.setElement(1, 1, pose2d.getCosTheta());

		log.debug("pose2d.getCosTheta(): " + pose2d.getCosTheta() + 
				", pose2d.getSinTheta(): " + pose2d.getSinTheta() + 
				", pose2d.getTheta(): " + pose2d.getTheta());
		log.debug("create trafo: R: " + MatrixUtil.toString(rot, 2, 4));
		this.rotT = (AbstractDoubleSquareMatrix) rot.transpose();
	}

	
	public RigidBodyTranformation2D(double x, double y, double theta)
	{
		this(new Pose2D(x, y, theta));
	}


	public RigidBodyTranformation2D rot(double thetaCam)
	{
		return new RigidBodyTranformation2D (new Pose2D(pose2d.getX(), pose2d.getY(), pose2d.getTheta() + thetaCam));
	}
	

	public Pose2D getAsPose()
	{
		return pose2d;
	}


	public Pose2D inverseRotatePosition(Pose2D pose)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, pose.getX());
		vectDisp.setComponent(1, pose.getY());

		vectDisp = rotT.multiply(vectDisp);

		return new Pose2D(vectDisp.getComponent(0), vectDisp.getComponent(1), pose.getTheta());
	}


	public AbstractDoubleVector inverseTrafoPointMaxDisp(AbstractDoubleSquareMatrix covar)
	{
		AbstractDoubleSquareMatrix  covPoint = MatrixUtil.subMatrix(covar, 0, 2);

		covPoint = rotT.multiply(covPoint).multiply(rot);

		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, Math.sqrt(covPoint.getElement(0, 0))); 
		vectDisp.setComponent(1, Math.sqrt(covPoint.getElement(1, 1)));

		return vectDisp;
	}


	public AbstractDoubleVector directTrafoLine(AbstractDoubleVector vectDisp)
	{
		vectDisp = rot.multiply(vectDisp);
		vectDisp = vectDisp.add(trans);
		return vectDisp;
	}


	public AbstractDoubleVector inverseTrafoLine(AbstractDoubleVector vectDisp)
	{
		vectDisp = vectDisp.subtract(trans);
		vectDisp = rotT.multiply(vectDisp);
		return vectDisp;
	}


	public AbstractDoubleVector inverseTrafoLine(VLineXY line)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, line.getX());
		vectDisp.setComponent(1, line.getY());

		vectDisp = vectDisp.subtract(trans);

		vectDisp = rotT.multiply(vectDisp);

		return vectDisp;
	}

//*
	public AbstractDoubleVector directTrafoLine(VLineXY line)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, line.getX());
		vectDisp.setComponent(1, line.getY());

		vectDisp = rot.multiply(vectDisp);
		vectDisp = vectDisp.add(trans);

		return vectDisp;
	}
//*/

	public AbstractDoubleVector inverseTrafoLine(VLineRef line)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, line.getX());
		vectDisp.setComponent(1, line.getY());

		vectDisp = vectDisp.subtract(trans);

		vectDisp = rotT.multiply(vectDisp);

		return vectDisp;
	}


	public AbstractDoubleVector directTrafoLine(VLineRef line)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, line.getX());
		vectDisp.setComponent(1, line.getY());

		vectDisp = rot.multiply(vectDisp);
		vectDisp = vectDisp.add(trans);

		return vectDisp;
	}


	public double inverseTrafoLineGetY(double x, double y)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, x);
		vectDisp.setComponent(1, y);

		vectDisp = vectDisp.subtract(trans);
		vectDisp = rotT.multiply(vectDisp);

		return vectDisp.getComponent(1);
	}


	public void directTrafoLineByRef(VLineXY line)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, line.getX());
		vectDisp.setComponent(1, line.getY());

		vectDisp = rot.multiply(vectDisp);
		vectDisp = vectDisp.add(trans);

		line.setX(vectDisp.getComponent(0));
		line.setY(vectDisp.getComponent(1));
	}


	public void inverseLine(double x, double y, double [] lineRet)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, x);
		vectDisp.setComponent(1, y);

		vectDisp = vectDisp.subtract(trans);
		vectDisp = rotT.multiply(vectDisp);

		lineRet[0] = (vectDisp.getComponent(0));
		lineRet[1] = (vectDisp.getComponent(1));
	}


	public void inverseRotateLine(double x, double y, double [] lineRet)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, x);
		vectDisp.setComponent(1, y);

		vectDisp = rotT.multiply(vectDisp);

		lineRet[0] = (vectDisp.getComponent(0));
		lineRet[1] = (vectDisp.getComponent(1));
	}
	public void directRotateLine(double x, double y, double [] lineRet)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, x);
		vectDisp.setComponent(1, y);

		vectDisp = rot.multiply(vectDisp);

		lineRet[0] = (vectDisp.getComponent(0));
		lineRet[1] = (vectDisp.getComponent(1));
	}


	public void directLine(double x, double y, double [] lineRet)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, x);
		vectDisp.setComponent(1, y);

		vectDisp = rot.multiply(vectDisp);
		vectDisp = vectDisp.add(trans);

		lineRet[0] = (vectDisp.getComponent(0));
		lineRet[1] = (vectDisp.getComponent(1));
	}


	public void directLine(double [] lineIn, double [] lineRet)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, lineIn[0]);
		vectDisp.setComponent(1, lineIn[1]);

		vectDisp = rot.multiply(vectDisp);
		vectDisp = vectDisp.add(trans);

		lineRet[0] = (vectDisp.getComponent(0));
		lineRet[1] = (vectDisp.getComponent(1));
	}


	public void inverseTrafoLineByRef(VLineXY line)
	{
		AbstractDoubleVector vectDisp = new DoubleVector(2);
		vectDisp.setComponent(0, line.getX());
		vectDisp.setComponent(1, line.getY());

		vectDisp = vectDisp.subtract(trans);

		vectDisp = rotT.multiply(vectDisp);

		line.setX(vectDisp.getComponent(0));
		line.setY(vectDisp.getComponent(1));
	}


	public String toString()
	{
		return "trafo: " + getAsPose().toString() + ", R: " + MatrixUtil.toString(rot, 2, 4);
	}
}



