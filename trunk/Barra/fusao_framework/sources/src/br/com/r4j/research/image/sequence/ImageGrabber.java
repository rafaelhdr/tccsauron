package br.com.r4j.research.image.sequence;



/** 
 * Classe centralizadora dos dados, repons�vel por gerar as estimativas.
 *
 * Tamb�m � a classe que coleta os dados. Para os dados ass�ncronos,
 * ela � chamada por algu�m. Para os dados dispon�veis sempre, ela
 * chama sincronamente.
 */
public interface ImageGrabber
{
	public void imageReceived(int [] inData, int [] outData, int width, int height, long imageTimestamp);
}
