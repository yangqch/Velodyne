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

import velodyne2d.LidarGLViewerForDetection;
import velodyne2d.LidarGLViewerForTracking;

import VelodyneDataIO.LidarFrameFactory;
import VelodyneView.AnimatorStopper;
import VelodyneView.LidarFrameProcessor;

import com.jogamp.opengl.util.FPSAnimator;

public class LidarViewerMain2D {
	// Define constants for the top-level container
	private static String TITLE = "3D Lidar";  // window's title
	private static final int CANVAS_WIDTH = 800;  // width of the drawable
	private static final int CANVAS_HEIGHT =1200; // height of the drawable
	private static final int FPS = 15; // animator's target frames per second
	
	static final float startTime=0; //138;//0;
	/** The entry main() method to setup the top-level container and animator */
	public LidarViewerMain2D(String dataset, ViewerType viewType, Properties conf) {
		LidarFrameFactory lfFac = null;
		LidarFrameProcessor processor = null;
		try{//raw velodyne data
			lfFac=new LidarFrameFactory(new File(dataset));
			
			//lfFac=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/AidedINS/realtime/data/VELODYNE_agg_raw_road_use_midstate_intensity.dat"));
//				lfFac=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/CA215N/trip1/mid_object_0.3_1.dat"));
//				lfFac=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/iowa_big/mid_object_0.3_1.dat"));
			//lfFac=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/frame.dat"));
			//GridmapMatrix<HeightCell> heightMatrix = GridmapMatrix.loadGridmapMatrix(new File("/home/qichi/Qichi_Velodyne/map/HeightMap/lidar_frame_lowest5"), new HeightCell(10), true);
			//GridmapMatrix<OccupyCell> cellMatrix = GridmapMatrix.loadGridmapMatrix(new File("/home/qichi/Qichi_Velodyne/map/OccupyMap/high_1_conn_0.3_3"), new OccupyCell(), true);
			
			processor = new LidarFrameProcessor(lfFac, null, null);
			processor.getReady(startTime);
			
		}catch(Exception e){
			e.printStackTrace();
			if(processor!=null) processor.stop();
			System.exit(-1);
		}
		GLCanvas canvas=null;
		if(viewType==ViewerType.detection){
			canvas = new LidarGLViewerForDetection(processor);
		}else{
			canvas = new LidarGLViewerForTracking(processor, conf);
		}
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
		final String filename = args[0];
		if(args[1].equals("-d")){
			final ViewerType viewType = ViewerType.detection;
			SwingUtilities.invokeLater(new Runnable() {
				@Override
				public void run() {
					new LidarViewerMain2D(filename, viewType, null);  // run the constructor
				}
			});
		}else if(args[1].equals("-t")){
			try{
				FileInputStream in = new FileInputStream(args[2]);
				final Properties conf = new Properties();
				conf.load(in);
				in.close();
				
				final ViewerType viewType = ViewerType.tracking;
				SwingUtilities.invokeLater(new Runnable() {
					@Override
					public void run() {
						new LidarViewerMain2D(filename, viewType, conf);  // run the constructor
					}
				});
			}catch(Exception e){
				e.printStackTrace();
				System.exit(0);
			}
		}else{
			System.err.println("USAGE: data_file -d(detection)/-t(tracking) [tracking conf]");
		}
		// Run the GUI codes in the event-dispatching thread for thread safety
		
	}
}

enum ViewerType {detection, tracking}; 

