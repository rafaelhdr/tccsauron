
package br.com.r4j.research.image.sequence;

import java.awt.geom.Point2D;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleMatrix;
import JSci.maths.DoubleMatrix;
import br.com.r4j.math.geom.Point3D;
import br.com.r4j.research.vline.VLine;
import br.com.r4j.research.vline.VLineProj;
import br.com.r4j.robosim.Pose2D;


/**
 * Não é mais usado!
 * Não é mais usado!
 * Não é mais usado!
 * Não é mais usado!
 *
 *
 * Localiza num array os pontos de mínimo local.
 *
 *
 *
 * A projeção é considerada como tendo x como profundidade, y como u e z como v.
 *
 *
 */
public class ProjectionModel
{
	private static Log log = LogFactory.getLog(ProjectionModel.class.getName());

	
	public ProjectionModel()
	{
	}


	public Point2D getProjection(CameraModel camModel, Point3D pt3D)
	{
		double u = pt3D.getY() * camModel.getFocusDistance() / pt3D.getX();
		double v = pt3D.getZ() * camModel.getFocusDistance() / pt3D.getX();
		u = camModel.getUAxisPixelCount() - u + camModel.getUAxisPixelCenter();
		v = camModel.getVAxisPixelCount() - v + camModel.getVAxisPixelCenter();

		Point2D.Double pt2D = new Point2D.Double(u, v);
		return pt2D;
	}


	public AbstractDoubleMatrix getProjectionError(CameraModel camModel, Point3D pt3D, AbstractDoubleMatrix covar3D)
	{
		AbstractDoubleMatrix covarRes = new DoubleMatrix(2, 2);

		double sigmaX = Math.sqrt(covar3D.getElement(0, 0));
		double sigmaY = Math.sqrt(covar3D.getElement(1, 1));
		double sigmaZ = Math.sqrt(covar3D.getElement(2, 2));

		double u = pt3D.getY() * camModel.getFocusDistance() / pt3D.getX();
		double v = pt3D.getZ() * camModel.getFocusDistance() / pt3D.getX();
		u = camModel.getUAxisPixelCount() - u + camModel.getUAxisPixelCenter();
		v = camModel.getVAxisPixelCount() - v + camModel.getVAxisPixelCenter();

		double sigma2U = sigmaY * camModel.getFocusDistance() / pt3D.getX() + sigmaX * (u - camModel.getUAxisPixelCenter()) / (pt3D.getX() + sigmaX);
		double sigma2V = sigmaZ * camModel.getFocusDistance() / pt3D.getX() + sigmaX * (v - camModel.getVAxisPixelCenter()) / (pt3D.getX() + sigmaX);

		covarRes.setElement(0, 0, sigma2U*sigma2U);
		covarRes.setElement(1, 1, sigma2V*sigma2V);

		return covarRes;
	}


	public VLineProj getProjection(CameraModel camModel, VLine line, Pose2D poseOffsetFromOrigin)
	{
//		return this.getProjection(camModel, poseOffsetFromOrigin.transform(line));
		return null;
	}


	/**
	 * O erro é porcamente calculado considerando que não existam covariâncias, e considerando
	 * dois efeitos distintos: o erro causado apenas pelo erro do eixo y, paralelo ao plano de
	 * imagem, considerando o valor do eixo x fixo. E o erro do eixo x, perpedicular ao plano
	 * de projeção, considerando o valor do eixo y fixo.
	 *
	 *
//		Alterar isso urgente! Ui ....
//		A linha agora é considerada como sendo não mais (x, y), mas sim (x, focus*y/x)! 
//		Adaptar carculo!!
	 *  
	 *
	 */
/*
	public VLineProj getProjection(CameraModel camModel, VLine line)
	{
		double u = line.getY() * camModel.getFocusDistance() / line.getX();
		u = camModel.getUAxisPixelCount() - u + camModel.getUAxisPixelCenter();

		double sigmaX = line.getCovar().getElement(0, 0);
		double sigmaY = line.getCovar().getElement(1, 1);
		double deltaSigmaY = camModel.getFocusDistance() / line.getX();
		double deltaSigmaX = (u - camModel.getUAxisPixelCenter()) / (line.getX() + sigmaX);
		double sigma2U = deltaSigmaY * sigmaY * deltaSigmaY + deltaSigmaX * sigmaX * deltaSigmaX;

		VLineProj lineProj = new VLineProj(u, sigma2U);
		return lineProj;
	}
//*/
}



