package br.com.r4j.gui;

import java.util.EventListener;



/**
 * Recebe eventos indicando a posição do mouse na imagem.
 */
public interface CanvasMouseListener extends EventListener
{
	public void mouseDragged(CanvasMouseEvent e);

	public void mouseMoved(CanvasMouseEvent e);

	public void mouseClicked(CanvasMouseEvent e);

	public void mouseEntered(CanvasMouseEvent e);

	public void mouseExited(CanvasMouseEvent e);

	public void mousePressed(CanvasMouseEvent e);

	public void mouseReleased(CanvasMouseEvent e);

	public void selectionPerformed(CanvasSelectionEvent e);
}

