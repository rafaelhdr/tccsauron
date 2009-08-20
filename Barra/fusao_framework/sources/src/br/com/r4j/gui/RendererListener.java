package br.com.r4j.gui;

import java.util.EventListener;



/**
 * Recebe eventos indicando a posi��o do mouse na imagem.
 */
public interface RendererListener extends EventListener
{
	/**
	 * Chamado cada vez que o fundo � refeito (Paint mode).
	 */
	public void imageUpdatePerformed(RendererEvent e);


	/**
	 * Chamado cada vez que todo o resto j� foi desenhado (Paint mode).
	 */
	public void updatePerformed(RendererEvent e);


	/**
	 * Chamado cada vez que � necess�rio redesenhar o objeto (XOR mode).
	 */
	public void render(RendererEvent e);


	/**
	 * Chamado cada vez que � necess�rio apagar o objeto (XOR mode).
	 */
	public void erase(RendererEvent e);


	/**
	 * @return
	 */
	public String getName();
}

