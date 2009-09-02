package br.com.r4j.math.matrix;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/**
 * Localiza num array os pontos de mínimo local.
 *
 *
 */
public class BlockMaximasLocalizer
{
	private static Log log = LogFactory.getLog(BlockMaximasLocalizer.class.getName());

	
	private int widthWindow = 3;
	private int heightWindow = 3;
	private HashMap mapValues = new HashMap();
	private boolean bStoreValues = false;


	public BlockMaximasLocalizer()
	{
		bStoreValues = true;
	}


	public void setWindowSize(int width, int height)
	{
		this.widthWindow = width;
		this.heightWindow = height;
	}
	public int getWindowSize()	{return this.widthWindow*this.heightWindow;}

/*
	private int treshold = 0;
	public void setTreshhold(int treshold)	{this.treshold = treshold;}
	public int getTreshhold()	{return this.treshold;}
//*/

	/**
	 * @param minValue valor mínimo que um máximo precisa ter para ser considerado.
	 * @returns uma lista com Points indicando os achados.
	 */
	public ArrayList findLocalMaximas(int [] arrayMap, int width, int height, int minValue)
	{
		ArrayList listMaximas = new ArrayList();
		if (bStoreValues)
			mapValues.clear();

		int cutWidthRight = widthWindow/2;
		int cutWidthLeft = widthWindow - cutWidthRight;
		int cutHeightUp = heightWindow/2;
		int cutHeightBottom = heightWindow - cutHeightUp;

		for (int j = 0; j < height; j += heightWindow) for (int i = 0; i < width; i += widthWindow)
		{
			int idxMax = j*width + i;
			int iHigher = i, jHigher = j;
			int higherVal = arrayMap[idxMax];

			int windowWidthEnd = i + widthWindow <= width ? widthWindow : width - i;
			int windowHeightEnd = j + heightWindow <= height ? heightWindow : height - j;

			for (int jj = 0; jj < windowHeightEnd; jj++) for (int ii = 0; ii < windowWidthEnd; ii++)
			{
				int indexImg = (j + jj)*width + (i + ii);
				if (higherVal < arrayMap[indexImg])
				{
					higherVal = arrayMap[indexImg];
					idxMax = indexImg;
					iHigher = i + ii;
					jHigher = j + jj;
				}
			}

			if (minValue < higherVal)
			{
				Point pt = new Point(iHigher, jHigher);
				listMaximas.add(pt);
				if (bStoreValues)
					mapValues.put(pt, new Integer(higherVal));
			}
		}
		return listMaximas;
	}


	public HashMap getMap()
	{
		return mapValues;
	}
}

