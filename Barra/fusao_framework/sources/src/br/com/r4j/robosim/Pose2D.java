package br.com.r4j.robosim;

import JSci.maths.AbstractDoubleVector;
import JSci.maths.DoubleVector;
import br.com.r4j.math.*;


/**
 * Armazena um pose no plano horizontal, com seu respectivo erro.
 *
 * Para calcular as tranformações, se utiliza de linearização sobre o
 * ponto, através das técnicas de linearização do EKF.
 *
 * Para isso, é considerado como vetor de variáveis as duas
 * posições da reta, mais as tr6es componentes do pose.
 *
 */
public class Pose2D
{
	private double x;
	private double y;
	private double theta;
	private double cosTheta;
	private double sinTheta;


	public Pose2D(double x, double y, double theta)
	{
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.cosTheta = Math.cos(theta);
		this.sinTheta = Math.sin(theta);
	}


	public Pose2D(AbstractDoubleVector values)
	{
		this.x = values.getComponent(0);
		this.y = values.getComponent(1);
		this.theta = values.getComponent(2);
		this.cosTheta = Math.cos(theta);
		this.sinTheta = Math.sin(theta);
	}


	public double getX()	{return this.x;}
	public double getY()	{return this.y;}
	public double getTheta()	{return this.theta;}
	public double getSinTheta()	{return this.sinTheta;}
	public double getCosTheta()	{return this.cosTheta;}
	public void setX(double a)	{this.x = a;}
	public void setY(double a)	{this.y = a;}
	public void setTheta(double a)	{this.theta = a; this.cosTheta = Math.cos(theta); this.sinTheta = Math.sin(theta);}

	public void setPose(double x, double y, double a)
	{
		this.x = x; this.y = y;
		this.theta = a; this.cosTheta = Math.cos(theta); this.sinTheta = Math.sin(theta);
	}


	public AbstractDoubleVector convert2vector()
	{
		AbstractDoubleVector vect = new DoubleVector(3);
		vect.setComponent(0, x);
		vect.setComponent(1, y);
		vect.setComponent(2, theta);
		return vect;
	}


	/**
	 * Retorna um Pose2D que é o caminho contrário desse pose. 
	 */
	public Pose2D negate()
	{
		double xNegated = -x;
		double yNegated = -y;
		double thetaNegated = -theta;
		return new Pose2D(xNegated, yNegated, thetaNegated);
	}


	public double getDistance(Pose2D poseAct)
	{
		double dx = x - poseAct.getX();
		double dy = y - poseAct.getY();
		return Math.sqrt(dx*dx + dy*dy);
	}


	public double getDistance(double xin, double yin)
	{
		double dx = x - xin;
		double dy = y - yin;
		return Math.sqrt(dx*dx + dy*dy);
	}


	public double getAngularDistance(double ang)
	{
		return FunctionsR.angularDist(ang, theta);
	}
	

	public double getAngularDistance(Pose2D poseAng)
	{
		return FunctionsR.angularDist(poseAng.getTheta(), theta);
	}
	

	public Pose2D add(Pose2D poseAdd)
	{
		Pose2D pnew = new Pose2D(x + poseAdd.getX(), y + poseAdd.getY(), theta + poseAdd.getTheta());
		return pnew;
	}


	public Pose2D add(double dx, double dy, double dTheta)
	{
		Pose2D pnew = new Pose2D(x + dx, y + dy, theta + dTheta);
		return pnew;
	}


	public Pose2D sub(Pose2D poseAdd)
	{
		Pose2D pnew = new Pose2D(x - poseAdd.getX(), y - poseAdd.getY(), theta - poseAdd.getTheta());
		return pnew;
	}


	public double dist2(Pose2D poseAdd)
	{
		double dx = x - poseAdd.getX();
		double dy = y - poseAdd.getY();
		return dy*dy + dx*dx;
	}


	public Pose2D sub(AbstractDoubleVector values)
	{
		Pose2D pnew = new Pose2D(x - values.getComponent(0), y - values.getComponent(1), theta - values.getComponent(2));
		return pnew;
	}


	public String toString()
	{
		return "(" + x + "," + y + "," + theta + ")";
	}
}

