/*
 * Created on Dec 22, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim.estimator;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class EstimationException extends RuntimeException
{

	/**
	 * 
	 */
	public EstimationException()
	{
		super();
	}

	/**
	 * @param message
	 */
	public EstimationException(String message)
	{
		super(message);
	}

	/**
	 * @param cause
	 */
	public EstimationException(Throwable cause)
	{
		super(cause);
	}

	/**
	 * @param message
	 * @param cause
	 */
	public EstimationException(String message, Throwable cause)
	{
		super(message, cause);
	}

}
