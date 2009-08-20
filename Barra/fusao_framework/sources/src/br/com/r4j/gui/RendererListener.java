package br.com.r4j.gui;

import java.util.EventListener;



/**
 * Recebe eventos indicando a posição do mouse na imagem.
 */
public interface RendererListener extends EventListener
{
	/**
	 * Chamado cada vez que o fundo é refeito (Paint mode).
	 */
	public void imageUpdatePerformed(RendererEvent e);


	/**
	 * Chamado cada vez que todo o resto já foi desenhado (Paint mode).
	 */
	public void updatePerformed(RendererEvent e);


	/**
	 * Chamado cada vez que é necessário redesenhar o objeto (XOR mode).
	 */
	public void render(RendererEvent e);


	/**
	 * Chamado cada vez que é necessário apagar o objeto (XOR mode).
	 */
	public void erase(RendererEvent e);


	/**
	 * @return
	 */
	public String getName();
}

