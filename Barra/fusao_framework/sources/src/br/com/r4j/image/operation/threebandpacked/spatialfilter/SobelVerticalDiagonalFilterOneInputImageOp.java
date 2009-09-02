package br.com.r4j.image.operation.threebandpacked.spatialfilter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedOneInputImageOp;


public class SobelVerticalDiagonalFilterOneInputImageOp implements ThreeBandPackedOneInputImageOp
{
	private static Log log = LogFactory.getLog(SobelVerticalDiagonalFilterOneInputImageOp.class.getName());

	private int [] filterX = null;
	private int bWidth = 0;
	private int bHeight = 0;

	private int bNegativeHalfWidth = 0;
	private int bNegativeHalfHeight = 0;
	private int bPositiveHalfWidth = 0;
	private int bPositiveHalfHeight = 0;

	private boolean bModule = true;
	private boolean bPositive = true;


	public SobelVerticalDiagonalFilterOneInputImageOp()
	{
	}


	public void setOnlyPositive()
	{
		bModule = false;
		bPositive = true;
	}
	public void setOnlyNegative()
	{
		bModule = false;
		bPositive = false;
	}
	public void setModule()
	{
		bModule = true;
	}


	public int operate(int [] inData, int [] outData, int imgWidth, int imgHeight)
	{
		int max = imgWidth * imgHeight;
		if (filterX == null || filterX.length != max)
		{
			filterX = new int [] {-2, 0, 2};
			this.bWidth = 3;

			this.bNegativeHalfWidth = bWidth/2;
			this.bPositiveHalfWidth = bWidth-bNegativeHalfWidth;
		}

		for (int j = 0; j < imgHeight; j++)
		{
			for (int i = bNegativeHalfWidth; i < imgWidth - bPositiveHalfWidth; i++)
			{
				int bCount = 0, indexImgBase = j*imgWidth + i;
				int red = 0, green = 0, blue = 0;
				for (int ii = -bNegativeHalfWidth; ii < bPositiveHalfWidth; ii++)
				{
					int indexImg = indexImgBase + ii;
					red += ((filterX[bCount]) * ((inData[indexImg]&REDMASK)>>REDSHIFT));
					green += ((filterX[bCount]) * ((inData[indexImg]&GREENMASK)>>GREENSHIFT));
					blue += ((filterX[bCount]) * ((inData[indexImg]&BLUEMASK)>>BLUESHIFT));
					bCount++;
				}

				if (red < 0) red = -red; if (red > 255) red = 255;
				if (green < 0) green = -green; if (green > 255) green = 255;
				if (blue < 0) blue = -blue; if (blue > 255) blue = 255;

				int valMedian = (blue + green + red) / 3;
				outData[indexImgBase] = ((int)valMedian<<REDSHIFT) | ((int)valMedian<<GREENSHIFT) | ((int)valMedian<<BLUESHIFT);
			}
		}
		return 0;
	}
}


