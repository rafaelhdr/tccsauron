package br.com.r4j.gui;

import java.util.EventListener;



/**
 * Recebe eventos indicando a posição do mouse na imagem.
 * @modelguid {2D4F72F1-0B2B-49B8-A60B-68003F38CF10}
 */
public interface RendererListener extends EventListener
{
	/**
	 * Chamado cada vez que o fundo é refeito (Paint mode).
	 * @modelguid {3C8D78FD-62EA-416C-9BDD-D040B777F2E5}
	 */
	public void imageUpdatePerformed(RendererEvent e);


	/**
	 * Chamado cada vez que todo o resto já foi desenhado (Paint mode).
	 * @modelguid {CA2A64DE-1C70-452E-A5C7-9804DF284811}
	 */
	public void updatePerformed(RendererEvent e);


	/**
	 * Chamado cada vez que é necessário redesenhar o objeto (XOR mode).
	 * @modelguid {6BD7CA37-9100-4A82-B69C-86E15019981A}
	 */
	public void render(RendererEvent e);


	/**
	 * Chamado cada vez que é necessário apagar o objeto (XOR mode).
	 * @modelguid {7A0146E6-5534-45A9-BFBD-0DEF303435FF}
	 */
	public void erase(RendererEvent e);


	/**
	 * @return
	 * @modelguid {51F7CCC8-7552-4F32-83A4-9A7D96F05F9B}
	 */
	public String getName();
}

