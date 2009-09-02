package br.com.r4j.research.image.sequence;



/** 
 * Classe centralizadora dos dados, reponsável por gerar as estimativas.
 *
 * Também é a classe que coleta os dados. Para os dados assíncronos,
 * ela é chamada por alguém. Para os dados disponíveis sempre, ela
 * chama sincronamente.
 */
public interface ImageGrabber
{
	public void imageReceived(int [] inData, int [] outData, int width, int height, long imageTimestamp);
}
