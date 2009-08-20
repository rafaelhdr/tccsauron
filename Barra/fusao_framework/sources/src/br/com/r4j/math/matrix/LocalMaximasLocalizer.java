package br.com.r4j.math.matrix;

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/**
 * Localiza num array os pontos de m�nimo local.
 *
 *
 */
public class LocalMaximasLocalizer
{
	private static Log log = LogFactory.getLog(LocalMaximasLocalizer.class.getName());

	
	private int widthWindow = 3;
	private int heightWindow = 3;


	public LocalMaximasLocalizer()
	{
	}


	public void setWindowSize(int width, int height)
	{
		this.widthWindow = width;
		this.heightWindow = height;
	}
	public int getWindowSize()	{return this.widthWindow*this.heightWindow;}


	/**
	 * @param minValue valor m�nimo que um m�ximo precisa ter para ser considerado.
	 * @returns uma lista com Points indicando os achados.
	 */
	public List findLocalMaximas(int [] arrayMap, int width, int height, int minValue)
	{
		ArrayList listMaximas = new ArrayList();

		int cutWidthRight = widthWindow/2;
		int cutWidthLeft = widthWindow - cutWidthRight;
		int cutHeightUp = heightWindow/2;
		int cutHeightBottom = heightWindow - cutHeightUp;

		for (int j = 0; j < height; j++)
		{
			for (int i = 0; i < width; i++)
			{
				int idxCenter = j*width + i;
				int higherVal = arrayMap[idxCenter];
				int idxHigher = idxCenter;

				int windowWidthBegin = i-cutWidthRight >= 0 ? -cutWidthRight : -i;
				int windowWidthEnd = i+cutWidthLeft <= width ? cutWidthLeft : width-i;

				int windowHeightBegin = j-cutHeightUp >= 0 ? -cutHeightUp : -j;
				int windowHeightEnd = j+cutHeightBottom <= height ? cutHeightBottom : height-j;

				for (int jj = windowHeightBegin; jj < windowHeightEnd; jj++) for (int ii = windowWidthBegin; ii < windowWidthEnd; ii++)
				{
					int indexImg = (j + jj)*width + i + ii;
					if (indexImg == idxCenter)
						continue;

					if (higherVal <= arrayMap[indexImg])
					{
						higherVal = arrayMap[indexImg];
						idxHigher = indexImg;
					}
				}
				if (idxHigher != idxCenter && higherVal == arrayMap[idxCenter])
				{
//					log.debug("...j = " + j + ", i = " + i);
					arrayMap[idxCenter]--;
				}
				else if (idxHigher == idxCenter && minValue < arrayMap[idxCenter])
				{
//					log.debug("!j = " + j + ", i = " + i);
//					log.debug("arrayMap[idxCenter] = " + arrayMap[idxCenter]);
					listMaximas.add(new Point(i, j));
				}
			}
		}
		return listMaximas;
	}
}

