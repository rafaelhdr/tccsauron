package br.com.r4j.m2v.view.tostringconverter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.m2v.view.converter.ConversionException;

/**
 * @author giord
 *
 * Define um Conversor M2V/V2M que trabalha sob um dado Locale.
 */
public class StringFormatConverter extends DoubleConverter
{
	private static Log log = LogFactory.getLog(StringFormatConverter.class.getName());

	private int minSpaces = 0;


	public StringFormatConverter()
	{
	}


	public StringFormatConverter(int minSpaces)
	{
		this.minSpaces = minSpaces;
	}


	public void setFormat(int minSpaces)
	{
		this.minSpaces = minSpaces;
	}


	protected String doConvertM2V(Object value) throws ConversionException
	{
		String strVal = value.toString();
		if (strVal.length() < minSpaces)
			for (int i = strVal.length(); i < minSpaces; i++)
				strVal = " " + strVal;
		return strVal;
	}
}
