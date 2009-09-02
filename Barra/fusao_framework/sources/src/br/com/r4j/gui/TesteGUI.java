/*
 * Created on Nov 27, 2005
 *
 * Window - Preferences - Java - Code Style - Code Templates
 */
package br.com.r4j.gui;

import javax.swing.JFrame;

/**
 * @author giord
 *
 * Window - Preferences - Java - Code Style - Code Templates
 * @modelguid {41872C6C-9F3B-44A8-B5EB-AD9D8358E006}
 */
public class TesteGUI extends JFrame {

	/** @modelguid {B9E88F64-6165-48D0-A092-680D73855FCC} */
	private javax.swing.JPanel jContentPane = null;

	/**
	 * This is the default constructor
	 * @modelguid {21BAD983-502D-4985-AAEC-1AC8354A6B12}
	 */
	public TesteGUI() {
		super();
		initialize();
	}
	/**
	 * This method initializes this
	 * 
	 * @return void
	 * @modelguid {C2B51C15-582D-4C95-90EF-1B504BDBF3DC}
	 */
	private void initialize() {
		this.setSize(300,200);
		this.setContentPane(getJContentPane());
		this.setTitle("JFrame");
	}
	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 * @modelguid {06932B6C-0485-4C58-8C81-51A68FBEA45D}
	 */
	private javax.swing.JPanel getJContentPane() {
		if(jContentPane == null) {
			jContentPane = new javax.swing.JPanel();
			jContentPane.setLayout(new java.awt.BorderLayout());
		}
		return jContentPane;
	}
}
