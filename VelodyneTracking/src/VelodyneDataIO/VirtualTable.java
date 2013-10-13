package VelodyneDataIO;

import java.awt.geom.Point2D;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import jogamp.graph.font.typecast.ot.table.Table;

import calibration.GroundPlaneDetector;

import com.jogamp.common.nio.Buffers;

public class VirtualTable {
	
	public static void main(String[] argv){

	}
	
	private float rotDegMin=-180;
	private float rotDegMax=180;
	private float vertDegMin=-15;
	private float vertDegMax=15;
	//resolution of rotation and vertical angle
	private float rotRes=0.5f;
	private float vertRes=0.5f;
	
	public static double DEG2RAD=Math.PI/180.0;
	public static double RAD2DEG=180.0/Math.PI;
	
	private int rows, cols;//size of table
	private double[][] distTable;
	private double[][] intensityTable;
	private Point3D[][] point3DTable;
	private int[] startIdx;
	private int validPointNum;
	private ArrayList[][] buffer;//buffer contain all the raw data in one slot before aggregation
	
	
	public static final float defaultNullDist=0;//the distance assigned to the slot which contains no data
	public static final float defaultNullIntensity=-1;//the distance assigned to the slot which contains no data
	private final int nearThres=3;//near points within this range will be filtered
	private final int pepperRowSize=3;
	private final int pepperColSize=3;

	//map the LidarFrame to virtual table
	static public void convertLidarFrame(LidarFrame lf, VirtualTable vt){
		vt.reset();
		float[] data = lf.getDataArray();
		int dataNum  = lf.getPointNum();
		for(int i=0;i<dataNum;i+=1){
			vt.put(data[i*3], data[i*3+1], data[i*3+2], lf.getIntensity(i));
		}
		vt.aggregate();
		vt.make3DPoints();
	}
	
	static public void convertLidarPoint3D(Point3D[] points, VirtualTable vt){
		vt.reset();
		
		int dataNum  = points.length;
		for(int i=0;i<dataNum;i+=1){
			vt.put((float)points[i].x, (float)points[i].y, (float)points[i].z, 0);
		}
		vt.aggregate();
		vt.make3DPoints();
	}
	
	public void preprocess(){
		//this.filterSaltPepper(pepperRowSize, pepperColSize);
		//this.findStartIdx();
	}
	
	public void preprocessLog() throws IOException{
		//this.interpolate(maxZeroPts);
		this.logDistanceTable(new FileOutputStream(new File("/home/qichi/dist_raw.log")));
		this.filterSaltPepper(pepperRowSize, pepperColSize);
		this.logDistanceTable(new FileOutputStream(new File("/home/qichi/dist_no_pepper.log")));
		//this.filterDistance(disFilterWinSize);
		this.logDistanceTable(new FileOutputStream(new File("/home/qichi/dist_filter.log")));
		
		this.findStartIdx();
	}
	
	public VirtualTable(){
		this.cols = (int) Math.ceil((rotDegMax - rotDegMin) / rotRes);
		this.rows = (int) Math.ceil((vertDegMax - vertDegMin) / vertRes);
		this.reset(); 
		this.validPointNum=0;
	}
	
	public VirtualTable(float rotRes, float vertRes){
		this.rotRes = rotRes;
		this.vertRes = vertRes;
		this.cols = (int) Math.ceil((rotDegMax - rotDegMin) / rotRes);
		this.rows = (int) Math.ceil((vertDegMax - vertDegMin) / vertRes);
		this.reset(); 
		//System.out.printf("table %d * %d\n", rows, cols);
		this.validPointNum=0;
	}
	
	public VirtualTable(float rotDegMin, float rotDegMax, float vertDegMin, float vertDegMax, float rotRes, float vertRes) {
		this.rotDegMin=rotDegMin;
		this.rotDegMax=rotDegMax;
		this.vertDegMin=vertDegMin;
		this.vertDegMax=vertDegMax;
		this.rotRes = rotRes;
		this.vertRes = vertRes;
		this.cols = (int) Math.ceil((rotDegMax - rotDegMin) / rotRes);
		this.rows = (int) Math.ceil((vertDegMax - vertDegMin) / vertRes);
		this.reset();
		this.validPointNum=0;
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////rendering//////////////////////////////////////////////
	
	//get ground data buffer for rendering
	public FloatBuffer getGroundDataBuffer(float startRotDeg, float endRotDeg, GroundPlaneDetector det){
		return this.getProcessedDataBuffer(startRotDeg, endRotDeg, this.startIdx, det.getGroundBoundaryIdx());
	}
	
	public FloatBuffer getGroundFitDataBuffer(float startRotDeg, float endRotDeg, GroundPlaneDetector det){
		return this.getProcessedDataBuffer(startRotDeg, endRotDeg, this.startIdx, det.getGroundFitIdx());
	}
	
	//get non-ground data buffer for rendering	
	public FloatBuffer getObjectDataBuffer(float startRotDeg, float endRotDeg, GroundPlaneDetector det){
		return this.getProcessedDataBuffer(startRotDeg, endRotDeg, det.getObjHeightMask(), 2);
	}
	
	public FloatBuffer getBoundaryBuffer(float startRotDeg, float endRotDeg, GroundPlaneDetector det){
		int[] groundBoundary=det.getGroundBoundaryIdx();
		int[] startVertIdx = new int[groundBoundary.length];
		for(int i=0;i<groundBoundary.length;i++){
			startVertIdx[i]=groundBoundary[i]==0 ? 0 : groundBoundary[i]-1;
		}
		
		return this.getProcessedDataBuffer(startRotDeg, endRotDeg, startVertIdx, groundBoundary);
	}
	
	public FloatBuffer getBoundaryEndVectorBuffer(float startRotDeg, float endRotDeg, GroundPlaneDetector det){
		int[] groundBoundary=det.getGroundBoundaryIdx();
		int[] startVertIdx = new int[groundBoundary.length];
		int[] endVertIdx = new int[groundBoundary.length];
		for(int i=0;i<groundBoundary.length;i++){
			if(groundBoundary[i]==0 || groundBoundary[i]==this.rows){
				startVertIdx[i]=0;
				endVertIdx[i]=0;
			}else{
				startVertIdx[i]=groundBoundary[i]-1;
				endVertIdx[i]= groundBoundary[i]+1;
			}
			
			if(endVertIdx[i] - startVertIdx[i]!=2 && endVertIdx[i] - startVertIdx[i]!=0)
				System.out.println(String.format("(%d, %d)", startVertIdx[i], endVertIdx[i]));
		}
		
		return this.getProcessedDataBuffer(startRotDeg, endRotDeg, startVertIdx, endVertIdx);
	}
	
	//a help func for querying the table elements for rendering
	//data in each column between corresponding startVertIdx and endVertIdx will be put in data buffer
	private FloatBuffer getProcessedDataBuffer(float startRotDeg, float endRotDeg, int[] startVertIdx, int[] endVertIdx){
		this.validPointNum=0;
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.rows*this.cols*3);
		
		int startCol = this.rotDeg2colIdx(startRotDeg);
		int endCol = this.rotDeg2colIdx(endRotDeg);
		
		for(int j=startCol;j<endCol;j++){
			for(int i=startVertIdx[j];i<endVertIdx[j];i++){
				Point3D p = this.point3DTable[i][j];
				if(p!=null){
					fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
					this.validPointNum++;
				}
			}
		}
		
		fb.rewind();
		return fb;
	}
	
	/**
	 * get data from rendering using mask
	 * @param startRotDeg
	 * @param endRotDeg
	 * @param objMask
	 * @param objCode
	 * @return
	 */
	private FloatBuffer getProcessedDataBuffer(float startRotDeg, float endRotDeg, int[][] objMask, int objCode){
		this.validPointNum=0;
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.rows*this.cols*3);
		
		int startCol = this.rotDeg2colIdx(startRotDeg);
		int endCol = this.rotDeg2colIdx(endRotDeg);
		
		for(int j=startCol;j<endCol;j++){
			for(int i=0;i<rows;i++){
				if(objMask[i][j]==objCode){
					Point3D p = this.point3DTable[i][j];
					fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
					this.validPointNum++;
					
				}
			}
		}
		
		fb.rewind();
		return fb;
	}
	/**
	 * get data from rendering using mask, render all if mask=null
	 * @param startRotDeg
	 * @param endRotDeg
	 * @param mask: only render mask[i][j]==true, render all if mask=null 
	 * @return
	 */
	public FloatBuffer getProcessedDataBuffer(float startRotDeg, float endRotDeg, boolean[][] mask){
		if(mask==null) return getAllDataBuffer();
		
		this.validPointNum=0;
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.rows*this.cols*3);
		
		int startCol = this.rotDeg2colIdx(startRotDeg);
		int endCol = this.rotDeg2colIdx(endRotDeg);
		
		for(int j=startCol;j<endCol;j++){
			for(int i=0;i<rows;i++){
				if(mask[i][j]){
					Point3D p = this.point3DTable[i][j];
					if(p==null) continue;
					fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
					this.validPointNum++;
				}
			}
		}
		
		fb.rewind();
		return fb;
	}


	public FloatBuffer getAllDataBuffer(){
		this.validPointNum=0;
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.rows*this.cols*3);
		for(int i=0;i<this.rows;i++){
			for(int j=0;j<this.cols;j++){
				Point3D p = this.point3DTable[i][j];
				if(p!=null){
					fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
					this.validPointNum++;
				}
			}
		}
		fb.rewind();
		return fb;
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	public int[] getStartIdx(){
		return this.startIdx;
	}
	
	public double getMinRot(){
		return this.rotDegMin;
	}
	
	public double getMaxRot(){
		return this.rotDegMax;
	}
	
	public int getRowNum(){
		return this.rows;
	}
	
	public int getColNum(){
		return this.cols;
	}
	public double getRotRes(){
		return this.rotRes;
	}
	public double getVertRes(){
		return this.vertRes;
	}
	public double getDistance(int row, int col){
		return this.distTable[row][col];
	}
	
	public int getDataNum(){
		return this.validPointNum;
	}
	
	public String getAttribute(){
		return String.format("%.1f, %.1f, %d, %.1f, %.1f, %d", this.rotDegMin, this.rotDegMax, this.cols, this.vertDegMin, this.vertDegMax, this.rows);
	}
	
	public Point3D[][] getLidarPoint3D(){
		return this.point3DTable;
	}

	//calculate 2d coordinates of element at row and col
	//x is xyDist, y is be z-up
	public Point2D.Double getVerticalPoint(int row, int col){
		double vertRad = this.rowIdx2vertRad(row);
		double distance = this.distTable[row][col];
		return new Point2D.Double(distance*Math.cos(vertRad), distance*Math.sin(vertRad));
	}
	
	public Point3D getPoint3D(int row, int col){
		return this.point3DTable[row][col];
	}
	/**
	 * get point3D where mask==true
	 * @param mask
	 * @return
	 */
	public Point3D[] getPoint3D(boolean[][] mask, boolean needMaxRange){
		ArrayList<Point3D> data = new ArrayList<Point3D>();
		for(int i=0; i<this.rows; i++){
			for(int j=0; j<this.cols; j++){
				if(this.point3DTable[i][j]!=null){
					if(getDistance(i, j)>=LidarFrame.MAX_RANGE){
						if(needMaxRange){
							data.add(this.point3DTable[i][j]);
						}
					}else{
						if((mask!=null && mask[i][j]) || mask==null){
							data.add(this.point3DTable[i][j]);
						}	
					}
				}
			}
		}
		return data.toArray(new Point3D[data.size()]);
	}
	/**
	 * make mask on points, mask=true where the point is 
	 * @param points
	 * @return mask, true-point on, false-no point on
	 */
	public boolean[][] makeMask(Point3D[] points){
		boolean[][] mask = new boolean[rows][cols];
		for(Point3D p: points){
			if(p==null) continue;
			TableIndex index = this.point3DToIndex(p);
			if(index!=null){
				mask[index.row][index.col]=true;
			}
		}
		return mask;
	}
	
	public double getIntensity(int row, int col){
		return this.intensityTable[row][col];
	}
	
	//FIXME: is vertRes/2 necessary?
	public float rowIdx2vertRad(int row){
		return (row*this.vertRes+vertDegMin+this.vertRes/2) * (float)DEG2RAD;
	}
	
	//FIXME: is rotRes/2 necessary?
	public float colIdx2rotRad(int col){
		return (col*this.rotRes + rotDegMin + this.rotRes/2) * (float)DEG2RAD;
	}
	
	public int rotDeg2colIdx(double rotDeg){
		double col = Math.floor((rotDeg-rotDegMin)/this.rotRes);
		return col>=cols ? cols-1 : (int)col;
		
	}
	
	public int vertDeg2rowIdx(double vertDeg){
		double row = Math.floor((vertDeg-vertDegMin)/this.vertRes);
		return row==rows ? rows-1 : (int)row;
	}
	
	public void reset(){
		this.distTable = new double[rows][cols];
		this.intensityTable = new double[rows][cols];
		this.point3DTable = new Point3D[rows][cols];
		this.buffer = new ArrayList[rows][cols];
	}
	
	public void logDistanceTable(FileOutputStream fs) throws IOException{
		fs.write("distance\n".getBytes());
		
		fs.write(this.getAttribute().getBytes());
		fs.write('\n');
		
		//col first
		for(int i=0; i<cols; i++){
			for(int j=0; j<rows; j++){
				fs.write(String.format("%.2f,", this.distTable[j][i]).getBytes());
			}
			fs.write('\n');
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 * aggregate the distance of points in each slot of distance/intensity table
	 */
	private void aggregate(){
		for(int i=0;i<this.rows;i++){
			for(int j=0;j<this.cols;j++){
				ArrayList<DistIntensity> slot = this.buffer[i][j];
				if(slot==null){
					this.distTable[i][j]=defaultNullDist;
					this.intensityTable[i][j]=defaultNullIntensity;
				}
				else{
					DistIntensity di = this.findMedian(slot);//use median
					this.distTable[i][j] = di.dist;
					this.intensityTable[i][j] = di.intensity;
				}
			}
		}
		this.buffer=null;
	}
	
	/**
	 * make 3d point table from distance table
	 */
	private void make3DPoints(){
		for(int i=0;i<this.rows;i++){
			for(int j=0;j<this.cols;j++){
				//rotVertTable to Point3D, null for distance 0
				double dist,xyDist,x,y,z, rotRAD, vertRAD;
				dist=this.distTable[i][j];
				if(dist==0){
					continue;
				}
				vertRAD = this.rowIdx2vertRad(i); //(row*this.vertRes+vertDegMin+this.vertRes/2) * (float)DEG2RAD;
				xyDist = dist * Math.cos(vertRAD);
				z = dist * Math.sin(vertRAD);//z-down positive value, but negative vertical degree
				
				rotRAD = this.colIdx2rotRad(j); //(col*this.rotRes + rotDegMin + this.rotRes/2) * (float)DEG2RAD;
				x = xyDist * Math.cos(rotRAD);
				y = xyDist * Math.sin(rotRAD);

				this.point3DTable[i][j]=new Point3D(x, y, z);
			}
		}
	}
	
	/**
	 * put new data from lidar frame into virtual table
	 * points with distance 0 or out of table angle range will be ignored
	 * @param x
	 * @param y
	 * @param z
	 * @param intensity
	 */  
	private void put(float x, float y, float z, float intensity){
		
		double dist = calcDist(x, y, z);
		//abandon the points stay too close to the sensor which might be noise
		//if(dist<this.nearThres) return;
		if(dist<LidarFrame.MIN_RANGE) return;
		
		//calculate rotDeg and vertDeg of x,y,z
		//then put in corresponding arrays
		double xyDist= Math.sqrt((x*x+y*y));
		
		double rotDeg;//rotation angle follow the x-y coordinate system
		if(x==0){
			rotDeg = y>0 ? 90 : -90;
		}else{
			rotDeg = (Math.atan2(y, x) * RAD2DEG);
		}		
		double vertDeg = (Math.atan2(z, xyDist) * RAD2DEG);
		//abandon the points out of angle range
		if(rotDeg>this.rotDegMax || rotDeg<this.rotDegMin || vertDeg>this.vertDegMax || vertDeg<this.vertDegMin){
			//System.out.printf(String.format("%.2f, %.2f\n", rotDeg, vertDeg));
			return;
		}
		int col = this.rotDeg2colIdx(rotDeg); //(int)Math.floor((rotDeg-rotDegMin)/this.rotRes); 
		int row = this.vertDeg2rowIdx(vertDeg); //(int)Math.floor((vertDeg-vertDegMin)/this.vertRes);
		
		if(this.buffer[row][col]==null) this.buffer[row][col]=new ArrayList<DistIntensity>();
		ArrayList<DistIntensity> slot = this.buffer[row][col];
		
		if(dist>LidarFrame.MAX_RANGE){
			slot.add(new DistIntensity(LidarFrame.MAX_RANGE, intensity));
		}else{
			slot.add(new DistIntensity(dist, intensity));
		}
		
	}
	
	private TableIndex point3DToIndex(Point3D p){
		float x=(float)p.x; float y=(float)p.y; float z=(float)p.z;
		double dist = calcDist(x,y,z);
		//abandon the points stay too close to the sensor which might be noise
		//if(dist<this.nearThres) return;
		if(dist<LidarFrame.MIN_RANGE) return null;
		
		//calculate rotDeg and vertDeg of x,y,z
		//then put in corresponding arrays
		double xyDist= Math.sqrt((x*x+y*y));
		
		double rotDeg;//rotation angle follow the x-y coordinate system
		if(x==0){
			rotDeg = y>0 ? 90 : -90;
		}else{
			rotDeg = (Math.atan2(y, x) * RAD2DEG);
		}		
		double vertDeg = (Math.atan2(z, xyDist) * RAD2DEG);
		//abandon the points out of angle range
		if(rotDeg>this.rotDegMax || rotDeg<this.rotDegMin || vertDeg>this.vertDegMax || vertDeg<this.vertDegMin){
			//System.out.printf(String.format("%.2f, %.2f\n", rotDeg, vertDeg));
			return null;
		}
		int col = this.rotDeg2colIdx(rotDeg); //(int)Math.floor((rotDeg-rotDegMin)/this.rotRes); 
		int row = this.vertDeg2rowIdx(vertDeg); //(int)Math.floor((vertDeg-vertDegMin)/this.vertRes);
		
		return new TableIndex(row, col);
	}
	
	public boolean isPointMasked(Point3D p, boolean[][] mask){
		float x=(float)p.x; float y=(float)p.y; float z=(float)p.z;
		double dist = calcDist(x,y,z);
		//abandon the points stay too close to the sensor which might be noise
		//if(dist<this.nearThres) return;
		if(dist<LidarFrame.MIN_RANGE) return false;
		
		//calculate rotDeg and vertDeg of x,y,z
		//then put in corresponding arrays
		double xyDist= Math.sqrt((x*x+y*y));
		
		double rotDeg;//rotation angle follow the x-y coordinate system
		if(x==0){
			rotDeg = y>0 ? 90 : -90;
		}else{
			rotDeg = (Math.atan2(y, x) * RAD2DEG);
		}		
		double vertDeg = (Math.atan2(z, xyDist) * RAD2DEG);
		//abandon the points out of angle range
		if(rotDeg>this.rotDegMax || rotDeg<this.rotDegMin || vertDeg>this.vertDegMax || vertDeg<this.vertDegMin){
			//System.out.printf(String.format("%.2f, %.2f\n", rotDeg, vertDeg));
			return false;
		}
		int col = this.rotDeg2colIdx(rotDeg); //(int)Math.floor((rotDeg-rotDegMin)/this.rotRes); 
		int row = this.vertDeg2rowIdx(vertDeg); //(int)Math.floor((vertDeg-vertDegMin)/this.vertRes);
		
		return mask[row][col];
	}
	
	private double calcDist(float x, float y, float z){
		return Math.sqrt((x*x+y*y+z*z));
	}
	
	
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////preprocessing////////////////////////////////////////////////
	
	private void findStartIdx(){
		this.startIdx = new int[cols];
		for(int i=0; i<cols; i++){
			int j=0;
			for(; j<rows; j++){
				if(this.distTable[j][i]!=0){
					this.startIdx[i]=j;
					break;
				}
			}
			if(j==rows) this.startIdx[i]=0;
		}
	}
	
	//filter out any single points
	private void filterSaltPepper(int row_thres, int col_thres){
		//vertical
		for(int i=0; i<cols; i++){
			int j=1;int num=0;
			for(; j<rows-1; j++){
				if(this.distTable[j][i]!=0){
					num++;
				}else{
					if(num>0){
						if(num<row_thres){
							for(int k=j-num-1;k<j;k++){
								this.distTable[k][i]=0;
							}
						}
						num=0;
					}
				}
			}
		}
		//horizontal
		for(int i=0; i<rows; i++){
			int j=1;int num=0;
			for(; j<cols-1; j++){
				if(this.distTable[i][j]!=0){
					num++;
				}else{
					if(num>0){
						if(num<col_thres){
							for(int k=j-num-1;k<j;k++){
								this.distTable[i][k]=0;
							}
						}
						num=0;
					}
				}
			}
		}
	}
	
	
	private double findMean(List<Double> array){
		int size = array.size();
		double sum=0;
		for(int i=0; i<size; i++){
			sum+=array.get(i);
		}
		return sum/size;
	}
	
	private DistIntensity findMedian(List<DistIntensity> array){
		Collections.sort(array);
		int size = array.size();
		if(size%2==0){
			if(array.get(size/2).dist > array.get(size/2-1).dist) return array.get(size/2-1); 
			else return array.get(size/2);
		}else{
			return array.get((size-1)/2);
		}
	}
	
	//get the minimum distance in each column
	public float getMinDistInCol(int colIdx, boolean[][] mask, boolean needMaxRange){
		double minDist = Float.MAX_VALUE;
		boolean isDet = false;
		for(int i=0; i<this.rows; i++){
			if(this.getDistance(i, colIdx)==LidarFrame.MAX_RANGE && needMaxRange){
				if(LidarFrame.MAX_RANGE<minDist){
					minDist = LidarFrame.MAX_RANGE;
					isDet = true;
					continue;
				}
			}
			if(mask!=null && !mask[i][colIdx]) continue;
			Point3D p = this.getPoint3D(i, colIdx);
			if (p==null) continue;
			double xyDist = Math.sqrt(p.x*p.x + p.y*p.y);
			if(xyDist<minDist){
				minDist = xyDist;
				isDet = true;
			}
		}
		if(!isDet) return -1;
		return (float)minDist;
	}
	
//	public VirtualScan2D toVirtualScan2D(){
//		VirtualScan2D vs2d = new VirtualScan2D(this.rotDegMin, this.rotDegMax, this.rotRes);
//		for(int i=0; i<this.cols; i++){
//			vs2d.setRay(i, this.getMinDistInCol(i));
//		}
//		return vs2d;
//	}
	
//	public VirtualScan2D toVirtualScan2D(boolean[][] mask){
//		VirtualScan2D vs2d = new VirtualScan2D(this.rotDegMin, this.rotDegMax, this.rotRes);
//		for(int i=0; i<this.cols; i++){
//			vs2d.setRay(i, this.getMinDistInCol(i, mask));
//		}
//		return vs2d;
//	}
}

class DistIntensity implements Comparable<DistIntensity>{
	double dist;
	float intensity;
	
	public DistIntensity(double d, float i) {
		dist = d;
		intensity = i;
	}

	@Override
	public int compareTo(DistIntensity o) {
		if(dist<o.dist) return -1;
		else if(dist>o.dist) return 1;
		else return 0;
	}
}


class TableIndex{
	int row;
	int col;
	public TableIndex(int row, int col) {
		this.row = row;
		this.col = col;
	}
}