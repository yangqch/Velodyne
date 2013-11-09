package velodyne2d;

import gridmap_generic.GridmapMatrix;
import gridmap_generic.HeightCell;
import gridmap_generic.OccupyCell;

import java.awt.Dimension;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import javax.media.opengl.GLAnimatorControl;
import javax.media.opengl.awt.GLCanvas;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import velodyne2d.LiDARGLViewerForReplay;
import velodyne2d.LidarGLViewer;
import velodyne2d.LidarGLViewerForDetection;
import velodyne2d.LidarGLViewerForTracking;

import VelodyneDataIO.LidarFrameFactory;
import VelodyneView.AnimatorStopper;
import VelodyneView.LidarFrameProcessor;


import com.jogamp.opengl.util.FPSAnimator;

public class LidarViewerMain2DReplay {
	// Define constants for the top-level container
	private static String TITLE = "3D Lidar";  // window's title
	private static final int CANVAS_WIDTH = 800;  // width of the drawable
	private static final int CANVAS_HEIGHT =1200; // height of the drawable
	private static final int FPS = 15; // animator's target frames per second
	
	
	/** The entry main() method to setup the top-level container and animator */
	public LidarViewerMain2DReplay(String dataFile, String trackFile) {
		LidarFrameFactory lfFac = null;
		LidarFrameProcessor processor = null;
		try{//raw velodyne data
			float startTime = 0;
			
			lfFac=new LidarFrameFactory(new File(dataFile));
			
			processor = new LidarFrameProcessor(lfFac, null, null);
			processor.getReady(startTime);
			
		}catch(Exception e){
			e.printStackTrace();
			if(processor!=null) processor.stop();
			System.exit(-1);
		}
		GLCanvas canvas=null;
		canvas = new LiDARGLViewerForReplay(processor, trackFile);
		
		canvas.setPreferredSize(new Dimension(CANVAS_WIDTH, CANVAS_HEIGHT));

		// Create a animator that drives canvas' display() at the specified FPS.
		final FPSAnimator animator = new FPSAnimator(canvas, FPS, true);

		// Create the top-level container
		final JFrame frame = new JFrame(); // Swing's JFrame or AWT's Frame
		frame.getContentPane().add(canvas);
		frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				// Use a dedicate thread to run the stop() to ensure that the
				// animator stops before program exits.
				new AnimatorStopper(animator).start();
			}
		});
		frame.setTitle(TITLE);
		frame.pack();
		frame.setVisible(true);
		animator.start(); // start the animation loop
	}
	
	public static void main(String[] args) {
		final String dataFile = args[0];
		final String trackFile = args[1];
		
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				new LidarViewerMain2DReplay(dataFile, trackFile);  // run the constructor
			}
		});
	}
	
}

