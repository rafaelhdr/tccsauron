/*
 * Created on Nov 30, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.commons.parser;

import java.io.IOException;
import java.io.Reader;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;


/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class TokenLineStreamParser
{
	private static Log log = LogFactory.getLog(TokenLineStreamParser.class.getName());

	protected int nextReaded = -1;
	protected boolean isNextAlreadyReaded = false;
	protected boolean reachedNewLine = false;
	
	protected boolean nextNonWhiteSpaceIs(char compare, Reader fRead) throws IOException
	{
		if (!isNextAlreadyReaded)
			nextReaded = (int) fRead.read();
		isNextAlreadyReaded = true;
		while (nextReaded == ' ' || nextReaded == '\t')
			nextReaded = (int) fRead.read();
		return nextReaded == compare;
	}


	protected int countNextsEnters(Reader fRead) throws IOException
	{
		int countEnter = 0;
		if (!isNextAlreadyReaded)
			nextReaded = (int) fRead.read();
		isNextAlreadyReaded = true;
		while (nextReaded == ' ' || nextReaded == '\t' || nextReaded == '\n' || nextReaded == '\r')
		{
			if (nextReaded == '\n')
				countEnter++;
			nextReaded = (int) fRead.read();
		}
//		log.debug("countNextsEnters:nextReaded = " + nextReaded);
		return countEnter;
	}


	protected String getWord(Reader fRead, char [] arrayDataBuffer) throws IOException
	{
		int counter = 0;
		if (!isNextAlreadyReaded)
			nextReaded = (int) fRead.read();
		else
			isNextAlreadyReaded = false;

		if (nextReaded == -1)
			return null;

		while ((nextReaded == ' ' || nextReaded == '\t' || nextReaded == '\r' || nextReaded == '\n') && nextReaded != -1)
			nextReaded = (int) fRead.read();
		while (nextReaded != ' ' && nextReaded != '\t' && nextReaded != '\r' && nextReaded != '\n' && nextReaded != -1)
		{
			arrayDataBuffer[counter++] = (char) nextReaded;
			nextReaded = (int) fRead.read();
		}
		if (nextReaded == -1)
			return null;

		if (nextReaded != '\r' || nextReaded != '\n')
			reachedNewLine = true;
		else
			reachedNewLine = false;

		return new String(arrayDataBuffer, 0, counter);
	}

	
	protected int getInt(Reader fRead, char [] arrayDataBuffer) throws IOException
	{
		String str = this.getWord(fRead, arrayDataBuffer);
//		log.debug("str: " + str);
		return Integer.parseInt(str);
	}


	protected double getDouble(Reader fRead, char [] arrayDataBuffer) throws IOException
	{
		String str = this.getWord(fRead, arrayDataBuffer).replace(',', '.');
//		log.debug("str: " + str);
		return Double.parseDouble(str);
	}


	protected char getChar(Reader fRead, char [] arrayDataBuffer) throws IOException
	{
		return this.getWord(fRead, arrayDataBuffer).charAt(0);
	}


	protected int getInt(String str) throws IOException
	{
//		log.debug("str: " + str);
		return Integer.parseInt(str);
	}


	/** @modelguid {6CA81708-0A62-40FA-B356-C2425CF3AAB1} */
	protected void jumpToNextLine(Reader fRead, char [] arrayDataBuffer) throws IOException
	{
		if (isNextAlreadyReaded && (nextReaded == '\r' || nextReaded == '\n' || nextReaded == -1))
			return;
		isNextAlreadyReaded = true;
		nextReaded = (int) fRead.read();
		while (!(nextReaded == '\r' || nextReaded == '\n' || nextReaded == -1))
			nextReaded = (int) fRead.read();
	}

}
