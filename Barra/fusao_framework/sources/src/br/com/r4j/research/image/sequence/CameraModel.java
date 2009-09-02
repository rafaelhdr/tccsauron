package br.com.r4j.research.image.sequence;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.*;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.research.vline.*;
import br.com.r4j.research.vline.VLineProj;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.commons.util.*;


/**
 * Localiza num array os pontos de mínimo local.
 *
 *
 */
public class CameraModel
{
	private static Log log = LogFactory.getLog(CameraModel.class.getName());

	
	// Centrado em zero.
//	private double visibilityRangeAngle = 37.56 * 180 / Math.PI;
	private double visibilityRangeAngle = 37.56 * Math.PI / 180;

	// em pixels.
	private double focusDistance = 450;
//	private double focusDistance = 1107;

	private int uAxisPixelCount = 320;
	private int vAxisPixelCount = 240;

	private int uAxisPixelCenter = 160;
	private int vAxisPixelCenter = 120;

	private double zRotCamera = 0;
	private double xOffset = 0;
	private double yOffset = 0;

	public CameraModel()
	{
	}



	public void setZRotCamera(double value)	{this.zRotCamera = value;}
	public double getZRotCamera()	{return this.zRotCamera;}

	public void setXOffset(double value)	{this.xOffset = value;}
	public double getXOffset()	{return this.xOffset;}

	public void setYOffset(double value)	{this.yOffset = value;}
	public double getYOffset()	{return this.yOffset;}

	public void setVisibilityRangeAngle(double value)	{this.visibilityRangeAngle = value;}
	public double getVisibilityRangeAngle()	{return this.visibilityRangeAngle;}

	public void setFocusDistance(int value)	{this.focusDistance = value;}
	public double getFocusDistance()	{return this.focusDistance;}

	public void setVFocusDistance(int value)	{this.focusDistance = value;}
	public double getVFocusDistance()	{return this.focusDistance;}

	public void setUFocusDistance(int value)	{this.focusDistance = value;}
	public double getUFocusDistance()	{return this.focusDistance;}

	public double getUFocusDistanceVariance()	{return focusDistance*0.04;}

	public void setUAxisPixelCount(int value)	{this.uAxisPixelCount = value;}
	public int getUAxisPixelCount()	{return this.uAxisPixelCount;}

	public void setVAxisPixelCount(int value)	{this.vAxisPixelCount = value;}
	public int getVAxisPixelCount()	{return this.vAxisPixelCount;}

	public void setUAxisPixelCenter(int value)	{this.uAxisPixelCenter = value;}
	public int getUAxisPixelCenter()	{return this.uAxisPixelCenter;}

	public void setVAxisPixelCenter(int value)	{this.vAxisPixelCenter = value;}
	public int getVAxisPixelCenter()	{return this.vAxisPixelCenter;}


	public VLineXY estimateLineIgnoreTheta(VLineProj p1, VLineProj p2, Pose2D poseP1, Pose2D poseP2)
	{
		Pose2D poseMoveRelative = poseP2.sub(poseP1);
		double uCenter = this.getUAxisPixelCenter();
		double u1 =  uCenter - p1.getU(), u2 = uCenter - p2.getU();

		double dX = poseMoveRelative.getX();
		double dY = poseMoveRelative.getY();
		double focalDist = this.getUFocusDistance();
		log.debug("u1 = " + u1 + ", u2 = " + u2 + ", dX = " + dX + ", dY = " + dY + ", focalDist = " + focalDist);
		double d2 = dX*dX + dY*dY;
		if (Math.abs(u1 - u2) > 1.2 || (d2 > 130*130))
		{
			double x = 0, y = 0;
			double u2_f = u2/focalDist, u1_f = u1/focalDist;

			if (Math.abs(u2_f - u1_f) > 0.00005)
			{
				y = u2_f*(
					u1_f*dX - dY
					)/(
					(u2_f - u1_f)
					);
				x = y/u2_f;
			}
			else
			{
				x = 1000*Math.abs(u2);
				y = u2_f*x;
			}

			log.debug("y = " + y + ", x = " + x);

			VLineXY line = new VLineXY(x, y, focalDist);
			return line;
		}
		else
		{
			log.debug("Movimento muito pequeno 1 ...");
			return null;
		}
	}


	/**
	 * Retorna uma linha com representação (depth, u) em relação a p2.
	 *
	 * Não calucal a covariância. 
	 *
	 * 
	 *
	 */
	public VLineXY estimateLine(VLineProj p1, VLineProj p2, Pose2D poseP1, Pose2D poseP2)
	{
		Pose2D poseMoveRelative = poseP2.sub(poseP1);
		double uCenter = this.getUAxisPixelCenter();
		double u1 =  uCenter - p1.getU(), u2 = uCenter - p2.getU();

// u2 - uCenter =  - p2.getU()
// p2.getU() = -(u2 - uCenter) = uCenter - u2;

		double dX = poseMoveRelative.getX();
		double dY = poseMoveRelative.getY();
		double dTheta = poseMoveRelative.getTheta();
		double d2 = dX*dX + dY*dY;
		double focalDist = this.getUFocusDistance();
		log.debug("u1 = " + u1 + ", u2 = " + u2 + ", dX = " + dX + ", dY = " + dY + ", dTheta = " + dTheta + ", focalDist = " + focalDist);
		if (Math.abs(u1 - u2) > 1.2 || (d2 > 130*130))
//		if (Math.abs(u1 - u2) > 2.5 || (d2 > 350*350))
		{
			double sinDTheta = poseMoveRelative.getSinTheta(), cosDTheta = poseMoveRelative.getCosTheta();
			log.debug("sinDTheta = " + sinDTheta + ", cosDTheta = " + cosDTheta);

			double x = 0, y = 0;
			double u2_f = u2/focalDist, u1_f = u1/focalDist;
/*
			x = (
				-u2_f*(cosDTheta*dX + sinDTheta*dY) + (cosDTheta*dY - sinDTheta*dX)
				)/(
				u1_f*(cosDTheta - sinDTheta*u2_f) - (sinDTheta + cosDTheta*u2_f)
				);
			x = (
				u1_f*dX - dY
				)/(
				cosDTheta*(u2_f - u1_f) + sinDTheta*(1 + u1_f*u2_f)
				);
			y = u2*x/focalDist;

			double y1 = u1_f*(
				u2_f*dX - dY
				)/(
				cosDTheta*(u1_f - u2_f) + sinDTheta*(1 + u1_f*u2_f)
				);
			double x1 = y1/u1_f;

			y = y1*(sinDTheta/u1_f + cosDTheta) + dY;
			x = y/u2_f;
/*/
			log.debug("u1_f*dX - dY: " + (u1_f*dX - dY));
			log.debug("cosDTheta*(u2_f - u1_f) + sinDTheta*(1 + u1_f*u2_f): " + (cosDTheta*(u2_f - u1_f) + sinDTheta*(1 + u1_f*u2_f)));
			log.debug("((u1_f*dX - dY)/(cosDTheta*(u2_f - u1_f) + sinDTheta*(1 + u1_f*u2_f)): " + ((u1_f*dX - dY)/(cosDTheta*(u2_f - u1_f) + sinDTheta*(1 + u1_f*u2_f))));
			y = u2_f*(
				u1_f*dX - dY
				)/(
				cosDTheta*(u2_f - u1_f) + sinDTheta*(1 + u1_f*u2_f)
				);
			x = y/u2_f;
//*/
			log.debug("y = " + y + ", x = " + x);

			VLineXY line = new VLineXY(x, y, focalDist);
			return line;
		}
		else
		{
			log.debug("Movimento muito pequeno 1 ...");
			return null;
		}
	}


	/**
	 * O erro de estimação de uma linha não considera o erro da posição absoluta do robô na posição em que a reta
	 * é representada como projeção, pois esse erro não interessa uma vez que a reta só pode ajudar a corrigir
	 * a movimentação do robô a partir do ponto em que ela foi detectada.
	 *
	 */
	public AbstractDoubleSquareMatrix estimateLineCovar(double xL, double yL, double u1, double u2, double dx, double dy, double cos_dt, double sin_dt, double f, double var_u1, double var_u2, AbstractDoubleSquareMatrix covar_dmov, int countSteps)
	{
		AbstractDoubleSquareMatrix covarInTriagleLine = new DoubleSquareMatrix(2 + 3);
		covarInTriagleLine.setElement(0, 0, var_u1);
		covarInTriagleLine.setElement(1, 1, var_u2);

		covarInTriagleLine.setElement(2, 2, covar_dmov.getElement(0, 0));
		covarInTriagleLine.setElement(2, 3, covar_dmov.getElement(0, 1));
		covarInTriagleLine.setElement(2, 4, covar_dmov.getElement(0, 2));
		covarInTriagleLine.setElement(3, 2, covar_dmov.getElement(1, 0));
		covarInTriagleLine.setElement(3, 3, covar_dmov.getElement(1, 1));
		covarInTriagleLine.setElement(3, 4, covar_dmov.getElement(1, 2));
		covarInTriagleLine.setElement(4, 2, covar_dmov.getElement(2, 0));
		covarInTriagleLine.setElement(4, 3, covar_dmov.getElement(2, 1));
		covarInTriagleLine.setElement(4, 4, covar_dmov.getElement(2, 2));
		log.debug("covarInTriagleLine: \r\n" + MatrixUtil.toString(covarInTriagleLine, 10, 3));

		AbstractDoubleMatrix tlJ = this.getTriangleLineJacobian(u1, u2, dx, dy, cos_dt, sin_dt, f);
		log.debug("tlJ: \r\n" + MatrixUtil.toString(tlJ, 10, 3));
		AbstractDoubleMatrix tlJ_t = (AbstractDoubleMatrix) tlJ.transpose();;

		AbstractDoubleMatrix covarOutTriagleLine = tlJ.multiply(covarInTriagleLine).multiply(tlJ_t);
		log.debug("covarOutTriagleLine: \r\n" + MatrixUtil.toString(covarOutTriagleLine, 10, 3));
		return MatrixUtil.convert2SquareMatrix(covarOutTriagleLine);

///////////////	
/*
		AbstractDoubleSquareMatrix covarOut = new DoubleSquareMatrix(2);
		covarOut.setElement(0, 0, covarOutTriagleLine.getElement(0, 0));
		covarOut.setElement(1, 1, var_u2);
		return MatrixUtil.convert2SquareMatrix(covarOut);
//*/
	}


	private AbstractDoubleMatrix getTriangleLineJacobian(double u1, double u2, double dx, double dy, double cos_dt, double sin_dt, double f)
	{
		AbstractDoubleMatrix j = new DoubleMatrix(2, 5);

		double u1f = u1/f;
		double u2f = u2/f;
		double V = u1f*dx - dy;
		double Z = cos_dt*(u2f - u1f) + sin_dt*(1 + u1f*u2f);
		double Z2 = Z*Z;

		double dXldU1 = (Z*dx/f - V*(sin_dt*u2f/f - cos_dt/f))/Z2;
		double dXldU2 = -V*(cos_dt/f + sin_dt*u1f/f)/Z2;
		double dXldDx = u1f/Z;
		double dXldDy = -1/Z;
		double dXldDth = -V*(cos_dt*(1 + u1f*u2f) - sin_dt*(u2f - u1f))/Z2;
/*
		double dXldU1 = -V*(cos_dt - sin_dt*u2f)/f/Z2;
		double dXldU2 = (-(sin_dt*dy + cos_dt*dx)*Z/f + (cos_dt + sin_dt*u1f/f)*V/f)/Z2;
		double dXldDx = (-u2f*cos_dt - sin_dt)/Z;
		double dXldDy = (cos_dt - u2f*sin_dt)/Z;
		double dXldDth = -V*(cos_dt*(1 + u1f*u2f) - sin_dt*(u2f - u1f))/Z2;
//*/
		double dYldU1 = dXldU1*u2f;
		double dYldU2 = (V*Z/f -V*u2f*(cos_dt/f + sin_dt*u1f/f))/Z2;
		double dYldDx = u2f*dXldDx;
		double dYldDy = u2f*dXldDy;
		double dYldDth = u2f*dXldDth;

		j.setElement(0, 0, dXldU1);
		j.setElement(0, 1, dXldU2);
		j.setElement(0, 2, dXldDx);
		j.setElement(0, 3, dXldDy);
		j.setElement(0, 4, dXldDth);

		j.setElement(1, 0, dYldU1);
		j.setElement(1, 1, dYldU2);
		j.setElement(1, 2, dYldDx);
		j.setElement(1, 3, dYldDy);
		j.setElement(1, 4, dYldDth);

		return j;
	}


	public AbstractDoubleSquareMatrix estimateLineMoveCovarCovar(double xL, double yL, double dx, double dy, double cos_dt, double sin_dt, AbstractDoubleSquareMatrix covar_dmov, AbstractDoubleSquareMatrix covar_line)
	{
		AbstractDoubleSquareMatrix covarInTriagleLine = new DoubleSquareMatrix(5);

		covarInTriagleLine.setElement(0, 0, covar_line.getElement(0, 0));
		covarInTriagleLine.setElement(0, 1, covar_line.getElement(0, 1));
		covarInTriagleLine.setElement(1, 0, covar_line.getElement(1, 0));
		covarInTriagleLine.setElement(1, 1, covar_line.getElement(1, 1));

		covarInTriagleLine.setElement(2, 2, covar_dmov.getElement(0, 0));
		covarInTriagleLine.setElement(2, 3, covar_dmov.getElement(0, 1));
		covarInTriagleLine.setElement(2, 4, covar_dmov.getElement(0, 2));
		covarInTriagleLine.setElement(3, 2, covar_dmov.getElement(1, 0));
		covarInTriagleLine.setElement(3, 3, covar_dmov.getElement(1, 1));
		covarInTriagleLine.setElement(3, 4, covar_dmov.getElement(1, 2));
		covarInTriagleLine.setElement(4, 2, covar_dmov.getElement(2, 0));
		covarInTriagleLine.setElement(4, 3, covar_dmov.getElement(2, 1));
		covarInTriagleLine.setElement(4, 4, covar_dmov.getElement(2, 2));
		log.debug("estimateLineMoveCovar:covarInTriagleLine: \r\n" + MatrixUtil.toString(covarInTriagleLine, 10, 3));

		AbstractDoubleMatrix tlJ = this.getMoveJacobian(xL, yL, dx, dy, cos_dt, sin_dt);
		log.debug("estimateLineMoveCovar:tlJ: \r\n" + MatrixUtil.toString(tlJ, 10, 3));
		AbstractDoubleMatrix tlJ_t = (AbstractDoubleMatrix) tlJ.transpose();;

		AbstractDoubleMatrix covarOutTriagleLine = tlJ.multiply(covarInTriagleLine).multiply(tlJ_t);
		log.debug("estimateLineMoveCovar:covarOutTriagleLine: \r\n" + MatrixUtil.toString(covarOutTriagleLine, 10, 3));

///////////////	

		return MatrixUtil.convert2SquareMatrix(covarOutTriagleLine);
	}


	private AbstractDoubleMatrix getMoveJacobian(double xL, double yL, double x1, double y1, double cos_t1, double sin_t1)
	{
		AbstractDoubleMatrix j = new DoubleMatrix(2, 5);

		j.setElement(0, 0, cos_t1);
		j.setElement(1, 0, sin_t1);
		j.setElement(0, 1, -sin_t1);
		j.setElement(1, 1, cos_t1);
		j.setElement(0, 2, 1);
		j.setElement(1, 2, 0);
		j.setElement(0, 3, 0);
		j.setElement(1, 3, 1);
		j.setElement(0, 4, -sin_t1*xL - cos_t1*yL);
		j.setElement(1, 4, sin_t1*yL - cos_t1*xL);

		return j;
	}

	
	public double estimateLineXY_PCovar(double xL, double yL, double rho, AbstractDoubleSquareMatrix covar_line)
	{
		log.debug("covar_line: \r\n" + MatrixUtil.toString(covar_line, 10, 3));

		AbstractDoubleMatrix tlJ = this.getXY_PJacobian(xL, yL,  rho);
		log.debug("estimateLineMoveCovar:tlJ: \r\n" + MatrixUtil.toString(tlJ, 10, 3));
		AbstractDoubleMatrix tlJ_t = (AbstractDoubleMatrix) tlJ.transpose();;

		AbstractDoubleMatrix covarOutTriagleLine = tlJ.multiply(covar_line).multiply(tlJ_t);
		log.debug("estimateLineMoveCovar:covarOutTriagleLine: \r\n" + MatrixUtil.toString(covarOutTriagleLine, 10, 3));

///////////////	

		return covarOutTriagleLine.getElement(0, 0);
	}


	private AbstractDoubleMatrix getXY_PJacobian(double xL, double yL, double rho)
	{
		AbstractDoubleMatrix j = new DoubleMatrix(1, 2);

		j.setElement(0, 0, 2*xL/rho);
		j.setElement(0, 1, 2*yL/rho);

		return j;
	}

}


