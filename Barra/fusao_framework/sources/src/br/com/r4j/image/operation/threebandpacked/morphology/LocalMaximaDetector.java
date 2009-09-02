package br.com.r4j.image.operation.threebandpacked.morphology;

import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedOneInputImageOp;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;


/** 
 * Filtro binarizador
 */
public class LocalMaximaDetector implements ThreeBandPackedOneInputImageOp
{

	private int bWidth = 0;
	private int bHeight = 0;

	private int bNegativeHalfWidth = 0;
	private int bNegativeHalfHeight = 0;
	private int bPositiveHalfWidth = 0;
	private int bPositiveHalfHeight = 0;

	private int minValue = 0;

	/**
	 * A matriz b tem as mesmas propriedades da matriz imagem
	 */
	public LocalMaximaDetector()
	{
		this.setWidth(5);
		this.setHeight(5);
		this.minValue = 128;
	}

	public int getMinValue()	{return minValue;}
	public void setMinValue(int minValue)	{this.minValue = minValue;}


	public int getWidth()	{return bWidth;}
	public void setWidth(int width)
	{
		this.bWidth = width;
		this.bNegativeHalfWidth = bWidth/2;
		this.bPositiveHalfWidth = bWidth-bNegativeHalfWidth;
	}


	public int getHeight()	{return bHeight;}
	public void setHeight(int height)
	{
		this.bHeight = height;
		this.bNegativeHalfHeight = bHeight/2;
		this.bPositiveHalfHeight = bHeight-bNegativeHalfHeight;
	}


	public int operate(int [] inData, int [] outData, int imgWidth, int imgHeight)
	{
		int cutWidthRight = bNegativeHalfWidth;
		int cutWidthLeft = bWidth - cutWidthRight;
		int cutHeightUp = bNegativeHalfHeight;
		int cutHeightBottom = bHeight - cutHeightUp;

		int imgCount = imgWidth*cutHeightUp;
		for (int j = cutHeightUp; j < imgHeight - cutHeightBottom; j++)
		{
			imgCount += cutWidthRight;
			for (int i = cutWidthRight; i < imgWidth - cutWidthLeft; i++)
			{
				int idxCenter = j*imgWidth + i;
				int higherVal = ThreeBandPackedUtil.getIntensity(inData[idxCenter]);
				int idxHigher = idxCenter;
				for (int jj = -cutHeightUp; jj < cutHeightBottom; jj++) for (int ii = -cutWidthRight; ii < cutWidthLeft; ii++)
				{
					int indexImg = ((j + jj)*imgWidth + i + ii);
					if (indexImg == idxCenter)
						continue;

					int tmpVal = ThreeBandPackedUtil.getIntensity(inData[indexImg]);
					if (higherVal <= tmpVal)
					{
						higherVal = ThreeBandPackedUtil.getIntensity(inData[indexImg]);
						idxHigher = indexImg;
					}
				}
				if (idxHigher != idxCenter && higherVal == inData[idxCenter])
				{
					inData[idxCenter]--;
				}
				else if (idxHigher == idxCenter && minValue <= higherVal)
				{
					outData[idxCenter] = ThreeBandPackedUtil.getPackedPixel(255, 255, 255);
				}
				else
				{
					outData[idxCenter] = 0;
				}
				imgCount++;
			}
			imgCount += cutWidthLeft;
		}
		return 0;
	}
}
