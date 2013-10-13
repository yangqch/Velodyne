package VelodyneView;
import gridmap_generic.GridmapMatrix;
import gridmap_generic.HeightCell;
import gridmap_generic.OccupyCell;
import gridmap_generic.QuadGridmap;

import java.awt.*;
import java.awt.event.*;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.Deque;
import java.util.LinkedList;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;
import javax.swing.filechooser.FileSystemView;
import javax.xml.crypto.dsig.spec.ExcC14NParameterSpec;

import prepocess.ConnCompFilter;
import prepocess.HeightMapFilter;
import prepocess.OccupyMapFilter;
import prepocess.RangeFilter;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

import calibration.BodyFrame;
import calibration.CoordinateFrame;
import calibration.FrameTransformer;
import calibration.GroundPlaneDetector;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.util.gl2.GLUT;

import static javax.media.opengl.GL.*;  // GL constants
import static javax.media.opengl.GL2.*; // GL2 constants

public class LidarFrameProcessor {
	private LidarFrameFactory lidarFrameFac;
	private LidarFrame lidarFrame;
	private VirtualTable virtualTable;
	private GroundPlaneDetector mDet;
	private boolean[][] mask=null;
	private boolean[][] mask1=null;
	private boolean[][] mask2=null;
	private GridmapMatrix<HeightCell> heightMatrix;
	private GridmapMatrix<OccupyCell> occupyMatrix;
	
	private Deque<VirtualTable> vtQueue;
	private Deque<Float> timeQueue;
	private Deque<CoordinateFrame> frameQueue;

	private HeightMapFilter heightMapFilter;
	private OccupyMapFilter occupyMapFilter;
	private ConnCompFilter compFilter;
	private RangeFilter rangeFilter;
	
	CoordinateFrame calibFrame = new CoordinateFrame(new double[] {0,0,0,
			0.9999800441981136, 0.00037833642524528284, 0.0036987820702358474,
			0.00037833642524528284, 0.9896323094339619, -0.10125933784433956,
			-0.003718152332877361, 0.10179006768396229, 0.9947925857075474});
	
	FrameTransformer mTransformer;
	
	static final File groundPlaneFile = new File("/home/qichi/Velodyne/ground_plane.log");
	FileOutputStream fs;
	
	protected boolean showRawData;
	
	private static final int rotMin=-180;
	private static final int rotMax=180;

	public LidarFrameProcessor(){}

	public LidarFrameProcessor(LidarFrameFactory lfFac, GridmapMatrix<HeightCell> heightMatrix, GridmapMatrix<OccupyCell> occupyMatrix){
		this.lidarFrameFac=lfFac;
		this.lidarFrame=null;
		mTransformer = new FrameTransformer();
		this.heightMatrix = heightMatrix;
		this.occupyMatrix = occupyMatrix;

		vtQueue = new LinkedList<VirtualTable>();
		timeQueue = new LinkedList<Float>();
		frameQueue = new LinkedList<CoordinateFrame>();
	}
	
	public void getReady(float startTime) throws Exception{
		try{
			this.lidarFrame = this.lidarFrameFac.getLidarFrame();
			while(this.lidarFrame.getTime()<startTime){
				this.lidarFrame = this.lidarFrameFac.getLidarFrame();
			}
			
			this.virtualTable = new VirtualTable();
			VirtualTable.convertLidarFrame(this.lidarFrame, this.virtualTable);
			
			this.heightMapFilter = new HeightMapFilter(virtualTable, mTransformer);
			this.occupyMapFilter = new OccupyMapFilter(this.virtualTable, mTransformer);
			this.compFilter = new ConnCompFilter(this.virtualTable);
			this.rangeFilter = new RangeFilter(this.virtualTable);


		}catch(EOFException e){
			System.out.printf("no frame later than startTime %.3f\n", startTime);
			throw e;
		}
	}
	
	public void stop(){
		try{
			this.lidarFrameFac.close();
		}catch(Exception e){
			e.printStackTrace();
		}
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	public void readNextFrame() throws Exception{
		this.lidarFrame = this.lidarFrameFac.getLidarFrame();
		this.virtualTable = new VirtualTable();
		VirtualTable.convertLidarFrame(this.lidarFrame, this.virtualTable);
	}
	
	public void findConnComp(double conn_comp_neighbor_thres, int conn_comp_size_thres) throws Exception{
		this.compFilter.findConnComp(this.virtualTable, null, conn_comp_neighbor_thres, conn_comp_size_thres);
	}
	
	public void filterStaticConnComp(double pcrt){
		
	}
	
	/**
	 * filter virtual table using height map
	 * erase connected component with size smaller than 5
	 * @throws Exception
	 */
	public void filterWithHeightGridmap(double low, double high) throws Exception{
		CoordinateFrame localWorldFrame = lidarFrame.getLocalWorldFrame();		
		QuadGridmap<HeightCell> quad = heightMatrix.makeQuadGridmap(localWorldFrame.getPosition().y, localWorldFrame.getPosition().x, LidarFrame.MAX_RANGE);
		if(quad==null){
			throw new Exception("no quadMap can be made");
		}
		this.heightMapFilter.filter(this.virtualTable, quad, low, high, localWorldFrame, true);//low height, high height, body frame, if log ground points
	}
	
	/**
	 * filter only by occupancy map
	 * @throws Exception
	 */
	public void filterWithOccupyGridmap(double occupyProb, boolean[][] mask) throws Exception{
		CoordinateFrame localWorFrame = lidarFrame.getLocalWorldFrame();
		QuadGridmap<OccupyCell> quad = occupyMatrix.makeQuadGridmap(localWorFrame.getPosition().y, localWorFrame.getPosition().x, LidarFrame.MAX_RANGE);
		if(quad==null){
			throw new Exception("no quadMap can be made");
		}
		this.occupyMapFilter.filter(this.virtualTable, quad, localWorFrame, occupyProb, mask);
	}
	
	//////////////////////////////////////////////////////////////////////////////
	
	public void nextRawFrame() throws Exception{
		//read one more raw lidar frame, using calibrated frame
		//this.lidarFrame = this.lidarFrameFac.getLidarFrame(true, calibFrame, mTransformer);
		this.lidarFrame = this.lidarFrameFac.getLidarFrame();
	
		this.virtualTable = new VirtualTable();
		VirtualTable.convertLidarFrame(this.lidarFrame, this.virtualTable);
		//this.virtualTable.preprocess();
		//if(this.compFilter==null) this.compFilter = new ConnComp(this.virtualTable);
		this.compFilter.findConnComp(this.virtualTable, null, 0.3, 10);
		this.mask = this.compFilter.getCompMask();//connected component
		//this.rangeFilter.filterX(this.virtualTable, -15, 5);
		//this.mask = this.rangeFilter.getMask();
	}
	
	public void nextFrameWithConnCompFilter(double conn_comp_neighbor_thres, int conn_comp_size_thres) throws Exception{
		this.lidarFrame = this.lidarFrameFac.getLidarFrame();
		
		this.virtualTable = new VirtualTable();
		VirtualTable.convertLidarFrame(this.lidarFrame, this.virtualTable);
		
		if(conn_comp_neighbor_thres>0 && conn_comp_size_thres>0){
			this.compFilter.findConnComp(this.virtualTable, null, conn_comp_neighbor_thres, conn_comp_size_thres);
			this.mask = this.compFilter.getCompMask();//connected component	
		}
	}
	
//	public void nextFrameForGroupPlane() throws Exception{		
//		this.lidarFrame = this.lidarFrameFac.getLidarFrame(true, calibFrame, mTransformer);
//
//		this.virtualTable = new VirtualTable();
//		VirtualTable.convertLidarFrame(this.lidarFrame, this.virtualTable);
//		this.virtualTable.preprocess();
//
//		this.mDet = new GroundPlaneDetector(this.virtualTable);
//		this.mDet.process();
//		this.mDet.logGroundPlane(fs);
//		fs.flush();
//	}
	

//	/**
//	 * occupancy map + connected component
//	 * @param static_prct
//	 * @throws Exception
//	 */
//	public void nextFrameWithOccupyGridMap(double static_prct) throws Exception{
//		this.readNextFrame();
//		BodyFrame body = lidarFrame.getBodyFrame();
//		
//		//connected component
//		this.compFilter.findConnComp(this.virtualTable, null, 0.3, 10);
//		this.mask1 = this.compFilter.getCompMask();//connected component
//		//static mask
//		QuadGridmap<OccupyCell> quad = occupyMatrix.makeQuadGridmap(body.getPosition().y, body.getPosition().x, LidarFrame.MAX_RANGE);
//		
//		this.occupyMapFilter.filter(this.virtualTable, quad, body, 0.5, this.mask1);
//		this.mask2 = this.occupyMapFilter.getMask();//static mask, true is occupied
//		
//		this.compFilter.filterStaticComp(static_prct, this.mask2);
//		this.mask = this.compFilter.getDynamicCompMask();
//	}

	
	/**
	 * read all frames within 1 sec in a queue
	 * make mask that contain data from frame 1 sec ago 
	 */
	public void nextDoubleFrame(double interval) throws Exception{
		LidarFrame lf = this.lidarFrameFac.getLidarFrame();
		VirtualTable vt = new VirtualTable();
		VirtualTable.convertLidarFrame(lf, vt);
		vtQueue.addLast(vt); timeQueue.addLast(lf.getTime()); frameQueue.addLast(lf.getLocalWorldFrame()); 
		do{//load lidar frame until exceeds interval
			//this.lidarFrameFac.getLidarFrame(true);
			lf = this.lidarFrameFac.getLidarFrame();
			vt = new VirtualTable();
			VirtualTable.convertLidarFrame(lf, vt);
			vtQueue.addLast(vt); timeQueue.addLast(lf.getTime()); frameQueue.addLast(lf.getLocalWorldFrame());
			//debug
			System.out.printf("enqueue size %d\n", vtQueue.size());
		}while(lf.getTime()-timeQueue.peekFirst() < interval);
		//the current lidar frame and vt are the last in queues, the main ones to use
		this.lidarFrame = lf; this.virtualTable = vt;
	
		//clear queue until the first frame is within interval
		CoordinateFrame prevBody=null; float prevTime=-1; VirtualTable prevTable=null; 
		while(this.lidarFrame.getTime()-timeQueue.peekFirst() > interval){
			prevTable=vtQueue.pollFirst(); prevTime=timeQueue.pollFirst(); prevBody=frameQueue.pollFirst();
			//debug
			System.out.printf("dequeue size %d\n", vtQueue.size());
		}

		//previous points in current body frame
		Point3D[] cur_prevPoints = mTransformer.transform4D(prevBody, this.lidarFrame.getLocalWorldFrame(), prevTable.getPoint3D(null, false));
		//make mask on current vt based on previous points(in current frame)
		this.mask1 = this.virtualTable.makeMask(cur_prevPoints);
		
		//make connected component mask on current table
		this.compFilter.findConnComp(this.virtualTable, null, 0.3, 10);
		this.mask2 = this.compFilter.getCompMask();
		//mask shows points in both mask
		//this.mask2 = this.maskAnd(this.mask1, staticMask);
		//mask the comp with small portion of static
		this.compFilter.filterStaticComp(0.5, this.mask1);
		//connected component in current frame filtered by static components
		this.mask = this.compFilter.getDynamicCompMask();//dynamic components with small portion of overlap
	}
	
//	/**
//	 * range Z + connComp
//	 * @param vt
//	 * @return
//	 */
//	private boolean[][] preprocess(VirtualTable vt){
//		RangeFilter rangeFilter=new RangeFilter(vt);
//		rangeFilter.filterZ(-1, 1);
//		boolean[][] mask1 = rangeFilter.getMask();
//		
//		ConnComp connComp = new ConnComp(vt);
//		connComp.findConnComp(mask1);
//		connComp.filterComp(10);
//		boolean[][] mask2 = connComp.getMask();
//		return mask2;
//	}
		
	public void logVirtualTable(){
		try{
			this.virtualTable.preprocessLog();
			mDet.logDistanceTable(new FileOutputStream(new File("/home/qichi/dist.log")));
			mDet.logVertAngleTable(new FileOutputStream(new File("/home/qichi/slope.log")));
			mDet.logBoundary(new FileOutputStream(new File("/home/qichi/boundary.log")));
			System.out.println("log finished");
		}catch(Exception e){
			System.err.println("log virtual table failed");
			System.err.println(e);
		}
	}
	
	public boolean[][] maskAnd(boolean[][] mask1, boolean[][] mask2){
		int rows=mask2.length; int cols = mask2[0].length;
		boolean[][] mask = new boolean[rows][cols];
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				mask[i][j] = mask1[i][j] && mask2[i][j];
			}
		}
		return mask;
	}
	
	public HeightMapFilter getHeightMapFilter(){
		return heightMapFilter;
	}
	
	public OccupyMapFilter getOccupyMapFilter(){
		return occupyMapFilter;
	}
	
	public ConnCompFilter getCompFilter(){
		return compFilter;
	}
	
	public LidarFrame getCurFrame() {
		return lidarFrame;
	}

	public VirtualTable getVirtualTable() {
		return virtualTable;
	}

	public GroundPlaneDetector getmDet() {
		return mDet;
	}

	public boolean[][] getMask() {
		return mask;
	}

	public boolean[][] getMask1() {
		return mask1;
	}
	
	public boolean[][] getMask2() {
		return mask2;
	}
}