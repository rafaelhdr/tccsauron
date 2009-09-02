package br.com.r4j.robosim.estimator;


public interface DataConsumer
{
	/**
	 * Indica a necessidade de processar e preparar novos dados.
	 *
	 */
	public void dataAvailable();
}
