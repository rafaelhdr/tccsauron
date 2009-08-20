package br.com.r4j.m2v.view.tostringconverter;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.m2v.view.converter.ConversionException;

/**
 * @author giord
 *
 * Define um Conversor M2V/V2M que trabalha sob um dado Locale.
 */
public class NumberFormatDoubleConverter extends DoubleConverter
{
	private static Log log = LogFactory.getLog(NumberFormatDoubleConverter.class.getName());

	private int maxFraction = 0;
	private int minZeroes = 0;
	private int minSpaces = 0;
	private NumberFormat numberFormat = null;


	public NumberFormatDoubleConverter()
	{
		numberFormat = new DecimalFormat("#########0.00");
	}


	public NumberFormatDoubleConverter(int maxFraction, int minZeroes, int minSpaces)
	{
		this.maxFraction = maxFraction;
		this.minZeroes = minZeroes;
		this.minSpaces = minSpaces;
		this.setFormat();
	}


	public void setFormat(int maxFraction, int minZeroes, int minSpaces)
	{
		this.maxFraction = maxFraction;
		this.minZeroes = minZeroes;
		this.minSpaces = minSpaces;
		this.setFormat();
	}


	private void setFormat()
	{
		StringBuffer strBuff = new StringBuffer();
		for (int i = minZeroes; i < minSpaces; i++)
			strBuff.append("#");
		for (int i = 0; i < minZeroes; i++)
			strBuff.append("0");
		strBuff.append(".");
		for (int i = 0; i < maxFraction; i++)
			strBuff.append("0");

		numberFormat = new DecimalFormat(strBuff.toString());
		numberFormat.setMaximumFractionDigits(2);
	}


	protected String doConvertM2V(Object value)// throws ConversionException
	{
		String strVal = numberFormat.format(value);
		if (strVal.length() < minSpaces)
			for (int i = strVal.length(); i < minSpaces; i++)
				strVal = " " + strVal;
		return strVal;
	}

	
	protected Double doConvertV2M(String value) throws ConversionException
	{
		return new Double(value);
	}
}
