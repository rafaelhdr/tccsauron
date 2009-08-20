/*
 * Created on Dec 9, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package br.com.r4j.robosim;

/**
 * @author giord
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public interface NoiseControl
{
	/**
	 * Retorna true se pode controlar o ruído padrão.
	 */
	public boolean canControlNoise();


	/**
	 * Retorna true se pode adicionar ruído espúrio.
	 */
	public boolean canAddRandomReadings();
	
	
	/**
	 * Ajusta o ruído do sensor alterando com base no valor padrão de ruído.
	 * 
	 * @param noiseScale indica o quanto o ruido deve ser ampliado em relação ao ruído padrão. 
	 * Valor 1 é neutro, 2, dobra, 0.5 é metade.
	 */
	public void setNoiseLevel(double noiseScale);


	/**
	 * Ajusta leituras aleatórias. Trabalha com porcentagem de leituras aleatórias e desvio da leitura.
	 * 
	 * @param noiseScale indica o quanto a leitura deve der errada em relação a leitura real. 
	 * Valor 0 é neutro, 1, dobra, 2, tres vezes.
	 * @param noiseFrequency indica a porcentagem de leituras erradas em relação as reais. 
	 * Valor: de 0 a 100, sendo 100 todas leituras erradas.
	 */
	public void setRandomLevel(double noiseScale, double noiseFrequency);


	/**
	 * @param noiseInfo
	 */
	public void setNoiseControlInfo(NoiseControlInfo noiseInfo);


	/**
	 * @return
	 */
	public NoiseControlInfo getNoiseControlInfo();
}
