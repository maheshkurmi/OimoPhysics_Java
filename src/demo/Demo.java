package demo;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JLayeredPane;
import javax.swing.JScrollPane;
import javax.swing.SwingUtilities;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.GLCapabilities;
import com.jogamp.opengl.GLEventListener;
import com.jogamp.opengl.GLProfile;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.FPSAnimator;

import demo.common.DemoMain;
import demo.gl2renderer.GL2DebugDraw;

public class Demo extends JFrame implements GLEventListener, MouseListener, MouseMotionListener, KeyListener,WindowListener, MouseWheelListener {
	
	 // Define constants for the top-level container
	   private static String TITLE = "Java Oimo Physics Engine Demo";  // window's title
	private static final int CANVAS_WIDTH = 640; // width of the drawable
	private static final int CANVAS_HEIGHT = 480; // height of the drawable
	private static final int FPS = 60; // animator's target frames per second
	GLCanvas canvas;
	JLabel lblInfo;
	// Create a animator that drives canvas' display() at the specified FPS.
    final FPSAnimator animator ;

    DemoMain test;
    
    private double DPI_FACTOR;
	// Setup OpenGL Graphics Renderer

		/** Constructor to setup the GUI for this Component */
	public Demo() {
		 GLCapabilities glcapabilities =new GLCapabilities( GLProfile.get(GLProfile.GL2) );
 	     glcapabilities.setDoubleBuffered(true);
 	     
 	     
 	    //   glcapabilities.setFBO(true);
 		    glcapabilities.setSampleBuffers(true);
 		   glcapabilities.setNumSamples(4);
			// setup the stencil buffer to outline shapes
	        glcapabilities.setStencilBits(1);
	    //    this.canvas = SystemUtilities.isWindows()?new WindowsGLcanvas(glcapabilities):new GLCanvas( glcapabilities );// new NewtCanvasAWT(window);
	        canvas = new GLCanvas(glcapabilities);
	        canvas.setPreferredSize(new Dimension(CANVAS_WIDTH, CANVAS_HEIGHT));
	      
	        lblInfo=new JLabel();
	        lblInfo.setOpaque(false);
	        JScrollPane sp=new JScrollPane(lblInfo);
//	        JLayeredPane pane = new JLayeredPane();
//	        pane.setLayer(lblInfo,2,0);
//	        pane.setLayer(canvas,1,0);
//	        pane.setPreferredSize(new Dimension(CANVAS_WIDTH, CANVAS_HEIGHT));
//      
	        this.add(canvas,BorderLayout.CENTER);
	        this.add(sp,BorderLayout.EAST);
        this.addWindowListener(new WindowAdapter() {
           @Override
           public void windowClosing(WindowEvent e) {
              // Use a dedicate thread to run the stop() to ensure that the
              // animator stops before program exits.
              new Thread() {
                 @Override
                 public void run() {
                    if (animator.isStarted()) animator.stop();
                    System.exit(0);
                 }
              }.start();
           }
        });
        animator = new FPSAnimator(canvas, FPS);
        canvas.addGLEventListener(this);
		//initUserInput
        canvas.addMouseListener(this);
        canvas.addMouseMotionListener(this);
        canvas.addMouseWheelListener(this);
        canvas.addKeyListener(this);
		test = new DemoMain(CANVAS_WIDTH, CANVAS_HEIGHT);
		
        this.setTitle(TITLE);
        this.pack();
        this.setVisible(true);
         animator.start(); // start the animation loop
		
	}

	
	/**
	 * Called back immediately after the OpenGL context is initialized. Can be used
	 * to perform one-time initialization. Run only once.
	 */
	@Override
	public void init(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2(); // get the OpenGL graphics context
//		gl.glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // set background (clear) color
//		gl.glClearDepth(1.0f); // set clear depth value to farthest
//		gl.glEnable(GL2.GL_DEPTH_TEST); // enables depth testing
//		gl.glDepthFunc(GL2.GL_LEQUAL); // the type of depth test to do
//		gl.glHint(GL2.GL_PERSPECTIVE_CORRECTION_HINT, GL2.GL_NICEST); // best perspective correction
//		gl.glShadeModel(GL2.GL_SMOOTH); // blends colors nicely, and smoothes out lighting
//		gl.glEnable(GL2.GL_LIGHT0);
		int[] arr=new int[1];
		gl.glGetIntegerv( GL2.GL_DEPTH_BITS, arr,0);
		System.out.println("depth bit ="+arr[0]);
		test.init(new GL2DebugDraw(gl));
		String s =test.getDemoInfo();
		s=s.replaceAll("\n", "<br>");
		lblInfo.setText("<html>"+s+"</html>");
		this.pack();
	}

	/**
	 * Call-back handler for window re-size event. Also called when the drawable is
	 * first set to visible.
	 */
	@Override
	public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
		test.setSize(width, height);
		DPI_FACTOR=drawable.getSurfaceWidth()/canvas.getWidth();
	}

	/**
	 * Called back by the animator to perform rendering.
	 */
	@Override
	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2(); // get the OpenGL 2 graphics context
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT); // clear color and depth buffers
	
		test.loop();
		
	
	}

	/**
	 * Called back before the OpenGL context is destroyed. Release resource such as
	 * buffers.
	 */
	@Override
	public void dispose(GLAutoDrawable drawable) {
	}


	@Override
	public void keyTyped(KeyEvent e) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void keyPressed(KeyEvent e) {
		test.keyPressed(e.getKeyCode());
	}


	@Override
	public void keyReleased(KeyEvent e) {
		test.keyReleased(e.getKeyCode());
		if("Q".equalsIgnoreCase(e.getKeyChar()+"")||"E".equalsIgnoreCase(e.getKeyChar()+"")) {
			String s =test.getDemoInfo();
			s=s.replaceAll("\n", "<br>");
			lblInfo.setText("<html>"+s+"</html>");
			this.pack();
		}
	}


	@Override
	public void mouseDragged(MouseEvent e) {
		test.mouseMoved(DPI_FACTOR*e.getX(),DPI_FACTOR*e.getY());
	}


	@Override
	public void mouseMoved(MouseEvent e) {
		test.mouseMoved(DPI_FACTOR*e.getX(),DPI_FACTOR*e.getY());
	}


	@Override
	public void mouseClicked(MouseEvent e) {
		//System.out.println(test.getDemoInfo());
	}


	@Override
	public void mousePressed(MouseEvent e) {
		if(e.getButton()==MouseEvent.BUTTON1)test.mouseLPressed();
		else if(e.getButton()==MouseEvent.BUTTON3)test.mouseRPressed();
		test.mouseMoved(DPI_FACTOR*e.getX(),DPI_FACTOR*e.getY());
	}


	@Override
	public void mouseReleased(MouseEvent e) {
		if(e.getButton()==MouseEvent.BUTTON1)test.mouseLReleased();
		else if(e.getButton()==MouseEvent.BUTTON3)test.mouseRReleased();
		test.mouseMoved(DPI_FACTOR*e.getX(),DPI_FACTOR*e.getY());
	}


	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}


	 /** The entry main() method to setup the top-level container and animator */
	   public static void main(String[] args) {
	      // Run the GUI codes in the event-dispatching thread for thread safety
	      SwingUtilities.invokeLater(new Runnable() {
	         @Override
	         public void run() {
	           Demo demo=new Demo();
	         }
	      });
	   }


	@Override
	public void windowOpened(WindowEvent e) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void windowClosing(WindowEvent e) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void windowClosed(WindowEvent e) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void windowIconified(WindowEvent e) {
		animator.pause();
	}


	@Override
	public void windowDeiconified(WindowEvent e) {
		animator.resume();
	}


	@Override
	public void windowActivated(WindowEvent e) {
		// TODO Auto-generated method stub
		animator.resume();
	}


	@Override
	public void windowDeactivated(WindowEvent e) {
		animator.pause();
	}


	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
		test.mouseScrolled(e.getWheelRotation());
	}
	 
}
