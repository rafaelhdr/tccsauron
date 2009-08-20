package br.com.r4j.gui;


import java.awt.BorderLayout;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.Image;

import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JWindow;

/**
*/
public class SplashScreen extends JWindow
{
   private static final int   m_iconsHeight = 30;
   private static final int   m_iconsWidth = 30;

   private int                m_splashWidth;
   private int                m_splashHeight;
   private ImageIcon          m_icon;                                   // splash image
   private ImageIcon[]        m_icons = new ImageIcon[100];             // product icons
   private JLabel             m_splashLabel;
   private JPanel             m_iconsPanel;
   private boolean            m_bKilled = false;
   private int                m_iconsIdx = 0;                           // number of product icons
   private boolean            m_bLayout = false;
   private boolean            m_bWaiting = false;
   

	public SplashScreen(Frame frame, Image image, final long tmWaiting)
	{
		this(frame, new ImageIcon(image));
		
		Thread th = new Thread(new Runnable()
		{
			public void run()
			{
				try
				{
					Thread.sleep(tmWaiting);
				}
				catch (Exception e) {}
				kill();				
			}
		}
		);
		th.start();
	}


	public SplashScreen(Frame frame, Image image)
	{
		this(frame, new ImageIcon(image));
	}


	/**
	*/
	public SplashScreen(Frame frame, ImageIcon image)
	{
		super((null == frame) ? new Frame() : frame);
		try
		{
			this.getContentPane().setLayout(new BorderLayout());
			this.changeSplashImage(image);
		}
		catch (Exception e)
		{
			System.out.println("Exception in SplashScreen constructor - " + e);
			// don't try to do anything else if there was a problem.
			return;
		}
		catch(Error e)
		{
			System.out.println("Error in SplashScreen constructor - " + e);
			// don't try to do anything else if there was a problem.
			return;
		}
/*
		// interrupt the thread if the mouse is clicked.
		MouseAdapter ma = new MouseAdapter()
		{
			public void mousePressed(MouseEvent me)
			{
				kill();
			}
		};

		//addMouseListener( ma );

		addKeyListener(new KeyAdapter()
		{
			public void keyPressed(KeyEvent e)
			{
				if(e.getKeyCode() == e.VK_SPACE)
					kill();
			}
		});
//*/

		// change the cursor to the wait cursor
		this.startWaitCursor();
	}


	public void setVisible(final boolean visible)
	{
		superSetVisible(visible);
	}


	private void superSetVisible(boolean visible)
	{
		super.setVisible(visible);
	}


	// Sets the dialog cursor to a wait cursor.
	private void startWaitCursor()
	{
		if(!m_bWaiting)
		{
			setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
			m_bWaiting = true;
		}
	}


	// Sets the cursor back to normal from a wait cursor.
	private void endWaitCursor()
	{
		if(m_bWaiting)
		{
			m_bWaiting = false;
			setCursor(Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR));
		}
	}


	/**
	 * Set the splash screen background, icons panel and redisplay window.
	 */
	synchronized public void changeSplashImage(final ImageIcon icon)
	{
		this.m_icon = icon;
		this.m_splashHeight = m_icon.getIconHeight();
		this.m_splashWidth = m_icon.getIconWidth();

		// set the window size based on the size of splash image
		this.setSize(m_splashWidth, m_splashHeight + m_iconsHeight + 12);

		// define splashlabel layout
		this.m_splashLabel = new JLabel(m_icon)
		{
			public void paintComponent(Graphics g)
			{
				super.paintComponent(g);
				int tleft = 76;
				int ttop = 133;
				Font f = g.getFont();
				Font newFont = new Font(f.getName(),f.getStyle(),f.getSize()-2);
				g.setFont(newFont);
				// g.drawString(Version.getDisplayVersion(), this.getInsets().left+tleft, this.getInsets().top+ttop); 
			}
		};
		this.displaySplash();
	}


	/**
	* Add text to the bottom of the splash screen.
	*/
	synchronized public void addText(final String msg )
	{
		m_iconsPanel.removeAll();

		// clear and setup the icon panel
		JLabel txtLabel = new JLabel(msg);//COMMENTED BY VINOD V

		Dimension d = txtLabel.getPreferredSize();
		d.height = m_iconsHeight;
		txtLabel.setPreferredSize(d);
		m_iconsPanel.add(txtLabel);

		// redisplay icon panel on splash window
		m_iconsPanel.doLayout();
		m_iconsPanel.repaint();
	}


	/**
	 * Add an icon to the bottom of the splash screen.
	 */
	synchronized public void addIcon(final ImageIcon icon )
	{
		int numIcons = m_splashWidth / m_iconsWidth;
		if (m_iconsIdx < numIcons)
		{
			m_icons[m_iconsIdx] = icon;
			m_iconsIdx++;
		}
		else
		{
			// move everything down one and then insert it
			for (int i=0; i<numIcons-1; i++)
				m_icons[i] = m_icons[i+1];
			m_icons[numIcons-1] = icon;
			m_bLayout = true;
		}
	}


	/**
	* Setup the window layout and display it.
	*/
	synchronized private void displaySplash()
	{
		System.err.println("displaySplash()");
		this.getContentPane().removeAll();      // remove all containers from window

		// add mainLabel to window
		this.getContentPane().add(m_splashLabel, BorderLayout.CENTER);
		this.pack();
		this.setSize(m_splashWidth, m_splashHeight + m_iconsHeight + 12);

		// Center the dialog over the window
		GUIFactory.centerWindow(this);
		this.setVisible(true);
	}


	/**
	* Kill the splash screen.
	*/
	synchronized public void kill()
	{
		System.err.println("kill()");
		if (!m_bKilled)
		{ 
			dispose();
			m_bKilled = true;

			// set the cursor back to the default cursor
			endWaitCursor();
		}
	}
}
