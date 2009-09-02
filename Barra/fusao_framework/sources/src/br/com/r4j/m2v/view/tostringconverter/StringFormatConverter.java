package br.com.r4j.m2v.view.tostringconverter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import br.com.r4j.m2v.view.converter.ConversionException;

/**
 * @author giord
 *
 * Define um Conversor M2V/V2M que trabalha sob um dado Locale.
 * @modelguid {61322519-B3A0-4D22-AA35-163FA5797D82}
 */
public class StringFormatConverter extends DoubleConverter
{
	/** @modelguid {BFDF2FC1-1F53-4735-B7C1-16CF9D960EA4} */
	private static Log log = LogFactory.getLog(StringFormatConverter.class.getName());

	/** @modelguid {05DF7621-6F60-4410-AE42-387C486DDC20} */
	private int minSpaces = 0;


	/** @modelguid {41F149FF-0200-458B-BDE7-84C0AE3DF2A6} */
	public StringFormatConverter()
	{
	}


	/** @modelguid {F642B381-8905-4A94-8F50-2F95AF75D099} */
	public StringFormatConverter(int minSpaces)
	{
		this.minSpaces = minSpaces;
	}


	/** @modelguid {39F73C32-FFC1-4352-B647-35A888C8B31B} */
	public void setFormat(int minSpaces)
	{
		this.minSpaces = minSpaces;
	}


	/** @modelguid {B999611D-A5A3-4BF4-A740-144918D446D4} */
	protected String doConvertM2V(Object value) throws ConversionException
	{
		String strVal = value.toString();
		if (strVal.length() < minSpaces)
			for (int i = strVal.length(); i < minSpaces; i++)
				strVal = " " + strVal;
		return strVal;
	}
}
