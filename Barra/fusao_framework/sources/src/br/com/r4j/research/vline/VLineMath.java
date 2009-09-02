
package br.com.r4j.research.vline;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;


/**
 *
 */
public class VLineMath
{
	private static Log log = LogFactory.getLog(VLineMath.class.getName());


	/**
	 * paginas de 141 a 144.
	 */
	public static boolean canBeTheSame(VLineXY lineEst1, VLineXY lineEst2)
	{
		AbstractDoubleSquareMatrix covarEst1 = lineEst1.getCovar();
		AbstractDoubleSquareMatrix covarEst2 = lineEst2.getCovar();

		double var = covarEst1.getElement(0, 0) + covarEst1.getElement(1, 1);
		var += covarEst2.getElement(0, 0) + covarEst2.getElement(1, 1);

		double d2 = getSqrDistance(lineEst1, lineEst2);

		log.debug("d2: " + d2 + ", var: " + var);

		return d2 < var;
	}

//*
	public static double getSqrDistance(VLineXY lineEst1, VLineXY lineEst2)
	{
		double dx = lineEst2.getX() - lineEst1.getX();
		double dy = lineEst2.getY() - lineEst1.getY();

		return dx*dx + dy*dy;
	}
//*/
/*
	public static VLine mean(VLine line1, VLine line2)
	{
		AbstractDoubleSquareMatrix S1 = line1.getCovar();
		AbstractDoubleSquareMatrix S2 = line2.getCovar();

		AbstractDoubleSquareMatrix S1plusS2 = S1.add(S2);
		AbstractDoubleSquareMatrix S1plusS2_inv = (AbstractDoubleSquareMatrix) S1plusS2.inverse();
		
		return new VLine(
			S2.multiply(S1plusS2_inv).multiply(line1.convert2vector()).add(
			S1.multiply(S1plusS2_inv).multiply(line2.convert2vector())
			),
			S2.multiply(S1plusS2_inv).multiply(S1)
			);
	}
//*/
 }

