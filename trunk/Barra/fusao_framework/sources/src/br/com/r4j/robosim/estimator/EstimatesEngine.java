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
	 * Restorna os poses do caminho do rob� para cada uma das itera��es, para cada um dos estimadores:
	 * [runIdx][estIdx][stepIdx]
	 *
	 */
	public AbstractDoubleVector [][][] getEstimatesMean();


	/**
	 * Restorna as covari�ncias poses do caminho do rob� para cada uma das itera��es.
	 * [runIdx][estIdx][stepIdx]
	 *
	 */
	public AbstractDoubleSquareMatrix [][][] getEstimatesCovariance();
}
