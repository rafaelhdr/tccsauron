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
	 * Retorna true se pode controlar o ru�do padr�o.
	 */
	public boolean canControlNoise();


	/**
	 * Retorna true se pode adicionar ru�do esp�rio.
	 */
	public boolean canAddRandomReadings();
	
	
	/**
	 * Ajusta o ru�do do sensor alterando com base no valor padr�o de ru�do.
	 * 
	 * @param noiseScale indica o quanto o ruido deve ser ampliado em rela��o ao ru�do padr�o. 
	 * Valor 1 � neutro, 2, dobra, 0.5 � metade.
	 */
	public void setNoiseLevel(double noiseScale);


	/**
	 * Ajusta leituras aleat�rias. Trabalha com porcentagem de leituras aleat�rias e desvio da leitura.
	 * 
	 * @param noiseScale indica o quanto a leitura deve der errada em rela��o a leitura real. 
	 * Valor 0 � neutro, 1, dobra, 2, tres vezes.
	 * @param noiseFrequency indica a porcentagem de leituras erradas em rela��o as reais. 
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
