package br.com.r4j.robosim.estimator;

import java.util.List;

import JSci.maths.AbstractDoubleVector;
import br.com.r4j.robosim.NoiseControlInfo;


/** @modelguid {7D8C1A54-8C1B-43E3-A3B9-0E47C301B0D1} */
public interface EstimatesEngine
{
	/**
	 * Processa numberOfTries vezes os dados, modificando as entradas.
	 *
	 */
	public void processData(int numberOfTries, NoiseControlInfo noiseInfo, EstimatorEngineListener estList);


	/**
	 * Restorna os poses do caminho do robô para cada uma das iterações, para cada um dos estimadores:
	 * [runIdx][estIdx][stepIdx]
	 *
	 */
//	public AbstractDoubleVector [][][] getEstimatesMean();

	public AbstractDoubleVector [] getRealValues();

	public long [] getTotalTimes();


	/**
	 * Restorna as covariâncias poses do caminho do robô para cada uma das iterações.
	 * [runIdx][estIdx][stepIdx]
	 *
	 */
//	public AbstractDoubleSquareMatrix [][][] getEstimatesCovariance();


	public List getEstimatorRendererInfos();


	public List getEstimators();
}
