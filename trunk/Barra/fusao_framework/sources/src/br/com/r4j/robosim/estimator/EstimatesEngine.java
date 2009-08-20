package br.com.r4j.robosim.estimator;

import JSci.maths.AbstractDoubleSquareMatrix;
import JSci.maths.AbstractDoubleVector;
import br.com.r4j.math.VectorScrambler;
import br.com.r4j.robosim.NoiseControlInfo;


public interface EstimatesEngine
{
	/**
	 * Processa numberOfTries vezes os dados, modificando as entradas.
	 *
	 */
	public void processData(int numberOfTries, NoiseControlInfo noiseInfo);


	/**
	 * Restorna os poses do caminho do robô para cada uma das iterações, para cada um dos estimadores:
	 * [runIdx][estIdx][stepIdx]
	 *
	 */
	public AbstractDoubleVector [][][] getEstimatesMean();


	/**
	 * Restorna as covariâncias poses do caminho do robô para cada uma das iterações.
	 * [runIdx][estIdx][stepIdx]
	 *
	 */
	public AbstractDoubleSquareMatrix [][][] getEstimatesCovariance();
}
