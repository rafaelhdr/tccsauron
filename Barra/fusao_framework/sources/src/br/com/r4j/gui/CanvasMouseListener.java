package br.com.r4j.gui;

import java.util.EventListener;



/**
 * Recebe eventos indicando a posição do mouse na imagem.
 * @modelguid {8AFCBA86-47B0-4578-9029-EEB66F29F899}
 */
public interface CanvasMouseListener extends EventListener
{
	/** @modelguid {04311660-502E-4FC6-BD98-59877BD22D47} */
	public void mouseDragged(CanvasMouseEvent e);

	/** @modelguid {304BC806-A252-4C19-BDC0-5102CACF0127} */
	public void mouseMoved(CanvasMouseEvent e);

	/** @modelguid {54F49FC1-7D67-4E31-8ABE-90C644FEDC0E} */
	public void mouseClicked(CanvasMouseEvent e);

	/** @modelguid {62A2738A-A845-4C15-AFED-9D82C5E7A7A1} */
	public void mouseEntered(CanvasMouseEvent e);

	/** @modelguid {10C486F1-0700-4132-AE02-46D6CE7E64BB} */
	public void mouseExited(CanvasMouseEvent e);

	/** @modelguid {921AC999-1E63-4307-891B-FBBBB6F0CF58} */
	public void mousePressed(CanvasMouseEvent e);

	/** @modelguid {EEA0A343-DD21-4A17-9444-7B13E23EDF7B} */
	public void mouseReleased(CanvasMouseEvent e);

	/** @modelguid {8D1E67A7-FA24-4FA9-8356-53F4216B9170} */
	public void selectionPerformed(CanvasSelectionEvent e);
}

