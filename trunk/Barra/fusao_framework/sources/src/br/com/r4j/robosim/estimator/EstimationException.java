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
 * @modelguid {C02CE147-2C09-44DB-85F0-4F0A2F1A1600}
 */
public class EstimationException extends RuntimeException
{

	/**
	 * 
	 * @modelguid {A7F72545-686A-4902-80D3-CA8754741AE0}
	 */
	public EstimationException()
	{
		super();
	}

	/**
	 * @param message
	 * @modelguid {511436E2-AE84-4103-AA53-486B65F0047D}
	 */
	public EstimationException(String message)
	{
		super(message);
	}

	/**
	 * @param cause
	 * @modelguid {DBDB095F-0D87-4803-A409-A914B61D4736}
	 */
	public EstimationException(Throwable cause)
	{
		super(cause);
	}

	/**
	 * @param message
	 * @param cause
	 * @modelguid {BA829325-D258-4853-8AFB-B8E77E04568A}
	 */
	public EstimationException(String message, Throwable cause)
	{
		super(message, cause);
	}

}
