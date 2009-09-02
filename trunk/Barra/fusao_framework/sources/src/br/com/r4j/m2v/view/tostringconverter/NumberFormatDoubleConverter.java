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
 * @modelguid {7FDA565A-1ADC-4011-B423-B89BBA60F2F3}
 */
public class NumberFormatDoubleConverter extends DoubleConverter
{
	/** @modelguid {9BD93DED-55BE-40D0-9A3B-2C4EE7AF09B6} */
	private static Log log = LogFactory.getLog(NumberFormatDoubleConverter.class.getName());

	/** @modelguid {A55DA4F1-41DD-44DA-A2DA-55BA4F405D0C} */
	private int maxFraction = 0;
	/** @modelguid {72FF0B52-E6B5-4572-AC5E-C75CD7A677BE} */
	private int minZeroes = 0;
	/** @modelguid {12481BBD-C895-4F7D-826A-D03668BE721B} */
	private int minSpaces = 0;
	/** @modelguid {BDC76360-DD85-4C50-A060-F2625EAE66B3} */
	private NumberFormat numberFormat = null;


	/** @modelguid {37031172-CDE9-4789-9AE1-BB582974A553} */
	public NumberFormatDoubleConverter()
	{
		numberFormat = new DecimalFormat("#########0.00");
	}


	/** @modelguid {AC3FD50F-4734-47F8-B853-50A4FBF83F5F} */
	public NumberFormatDoubleConverter(int maxFraction, int minZeroes, int minSpaces)
	{
		this.maxFraction = maxFraction;
		this.minZeroes = minZeroes;
		this.minSpaces = minSpaces;
		this.setFormat();
	}


	/** @modelguid {4C6898DD-5C49-4953-9F8D-86BBEC50F335} */
	public void setFormat(int maxFraction, int minZeroes, int minSpaces)
	{
		this.maxFraction = maxFraction;
		this.minZeroes = minZeroes;
		this.minSpaces = minSpaces;
		this.setFormat();
	}


	/** @modelguid {0F502606-2D56-4BD8-91A8-B7DF421A6B56} */
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


	/** @modelguid {D0223B35-0E85-4C6C-9E8D-DEE15C139AEC} */
	protected String doConvertM2V(Object value) throws ConversionException
	{
		String strVal = numberFormat.format(value);
		if (strVal.length() < minSpaces)
			for (int i = strVal.length(); i < minSpaces; i++)
				strVal = " " + strVal;
		return strVal;
	}

	
	/** @modelguid {AD879579-8608-44FC-B6D8-E2E4025A6D5A} */
	protected Double doConvertV2M(String value) throws ConversionException
	{
		return new Double(value);
	}
}
