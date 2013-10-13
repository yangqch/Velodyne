package VelodyneView;

import java.awt.Dimension;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;

import javax.media.opengl.awt.GLCanvas;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import VelodyneDataIO.LidarFrameFactory;

import com.jogamp.opengl.util.FPSAnimator;

public class DoubleLidarViewerMain {
	// Define constants for the top-level container
		private static String TITLE = "3D Lidar";  // window's title
		private static final int CANVAS_WIDTH = 1600;  // width of the drawable
		private static final int CANVAS_HEIGHT =800; // height of the drawable
		private static final int FPS = 15; // animator's target frames per second
		
		static final float startTime=20;
		/** The entry main() method to setup the top-level container and animator */
		public DoubleLidarViewerMain() {
			LidarFrameFactory lfFac = null;
			DoubleLidarFrameProcessor processor = null;
			try{
				//LidarFrameFactory lfFac1=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/columbia/loop1/mid_object_0.3_1.dat"));
//				LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/columbia/loop5/low_object_0_0.3.dat"));
				//LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/columbia/loop1/dynamic_08.dat"));
				//LidarFrameFactory lfFac1=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/parkinglot/background/mid_object_0.3_1.dat"));
				//LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/parkinglot/car/low_object_0_0.3.dat"));
				//LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/parkinglot/background/dynamic_07.dat"));
				
				LidarFrameFactory lfFac1=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/iowa_big/mid_object_0.3_1.dat"));
				LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/iowa_big/low_object_0_0.3.dat"));
//				LidarFrameFactory lfFac1=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/iowa_big/frame.dat"));
//				LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/iowa_big/vt.dat"));
				
//				LidarFrameFactory lfFac1=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/parkinglot/car/mid_object_0.3_1.dat"));
//				LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/parkinglot/car/low_object_0_0.3.dat"));
				//LidarFrameFactory lfFac2=new LidarFrameFactory(new File("/home/qichi/Qichi_Velodyne/processed_data/parkinglot/car/dynamic_05.dat"));
				//lfFac2.getLidarFrame();
				processor = new DoubleLidarFrameProcessor(lfFac1, lfFac2);
				processor.getReady(startTime);
			}catch(Exception e){
				e.printStackTrace();
				if(processor!=null) processor.stop();
				System.exit(-1);
			}
			
			final GLCanvas canvas = new DoubleLidarGLViewer(processor);
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
			// Run the GUI codes in the event-dispatching thread for thread safety
			SwingUtilities.invokeLater(new Runnable() {
				@Override
				public void run() {
					new DoubleLidarViewerMain();  // run the constructor
				}
			});
		}
}

