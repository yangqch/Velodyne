package calibration;

import java.awt.geom.Point2D;
import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.LidarFrameFactory;
import VelodyneDataIO.Point3D;
import VelodyneDataIO.VirtualTable;

import com.jogamp.common.nio.Buffers;

public class GroundPlaneDetector {
	
	public static void main(String[] argv){
		try{
			DataOutputStream dos = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(new File("/home/qichi/Qichi_Velodyne/processed_data/low_points_lidar_frame.dat"))));
			File inputFile = new File("/home/qichi/Qichi_Velodyne/AidedINS/realtime/data/VELODYNE_agg_raw_road_use_midstate_intensity.dat");
			try{	
				LidarFrameFactory lfFac=new LidarFrameFactory(inputFile);
				VirtualTable vt = new VirtualTable();
				lfFac.getLidarFrame();
				
				int cnt=0;
				LidarFrame lf;
				CoordinateFrame groundFrame;
				while(true){
					lf = lfFac.getLidarFrame();
					VirtualTable.convertLidarFrame(lf, vt);
					vt.preprocess();

					GroundPlaneDetector det = new GroundPlaneDetector(vt);
					det.process();
					
					//log ground
					//$_time_G_12f_L_12f
					dos.writeByte('$');
					dos.writeFloat(lf.getTime());
					groundFrame = det.gpa.getGroundFrameInWorld(lf.getLocalWorldFrame());
					//dos.writeByte('G');
					groundFrame.printBinaryToFile(dos);//3+9 bytes
					//dos.writeByte('L');
					lf.getLocalWorldFrame().printBinaryToFile(dos);//3+9 bytes
					//lf.getBodyFrame().printAttitudeBinaryToFile(dos);
					
					//det.logMask3DLidarPointWithIntensity(dos, 1);
					//det.logMask3DPlanePointWithIntensity(dos, 2);
					det.logRawData(dos, lf, 0.2);
					
					cnt++;					
					if(cnt%100==0){
						dos.flush();
					}
					
				}
			}catch(EOFException e){
				System.out.println(e);
				return;
			}catch(IOException e){
				System.out.println(e);
			}finally{
				try{
					dos.close();
				}catch(Exception e){
					
				}
				System.out.println("finish");
			}
		}catch(Exception e){
			System.err.println(e);
			return;
		}
		
	}
	
	final static double angleGroundPointThres = Math.PI*0.85;//threshold for ground point which cannot be object
	final static double angleGroundFitThres = Math.PI*0.99;//threshold for points which will be fed to ground fitting
	final static int angleFilterWidth=3; //max non-zero points on row to be identified as noise
	final static int angleFilterHeight=3;//max non-zero points on col to be identified as noise
	final static double objLow=0.3;//low object to be filtered out
	final static double objHigh=2; //high object to be filtered out
	final static double localPlaneXrange = 5;//range of ground points passed to gpa
	final static double localPlaneYrange = 20;//range of ground points passed to gpa
	final static double angleObjThres = Math.PI/4;//in modifyBoundary, any angle larger than this threshold will not be considered as ground
	
	private VirtualTable vtable;       //pointer to virtual table
	private double[][] vertAngleTable;//store the angle diff around each point along the column
	private int rows, cols;//rows and cols for vertAngleTable
	private int[] groundFitIdx;//store row idx of first non-ground point, used for plane fitting, smaller criteria
	private int[] groundEndPointIdx;  //store row idx of first non-ground point, used for object filtering, larger criteria
	
	private Point3D[][] plane3DPoints;//3d point w.r.t ground plane in GroundPlanePCA
	private int[][] objHeightMask;    //0-ground, 1-low, 2-mid, 3-high
	private int[] objNum;//point number 0-ground, 1-low, 2-mid, 3-high
	
	private GroundPlanePCA gpa;      //pointer to GroundPlanePca
	private List<Point3D> groundPoints = new ArrayList<Point3D>();//ground points passed to gpa
	
	public GroundPlaneDetector(VirtualTable vt) {
		this.vtable = vt;
		rows = vt.getRowNum()-2;
		cols = vt.getColNum();
		
		this.reset();
		
		this.gpa = new GroundPlanePCA();
		gpa.setGridRange(-5, 5, 10, -10, 10, 10);
	}
	
	public int[] getGroundBoundaryIdx(){
		return this.groundEndPointIdx;
	}
	
	public int[] getGroundFitIdx(){
		return this.groundFitIdx;
	}
	
	public int[][] getObjHeightMask(){
		return this.objHeightMask;
	}
	
	public void process(){
		this.reset();
		//calculate the angle between each vectors
		this.makeVertAngleTable();
		//find the first non-ground points
		this.findBoundary(groundFitIdx, angleGroundFitThres);
		//fit plane
		this.fitGroundPlane(this.vtable.getStartIdx(), this.groundFitIdx);
		//transform points from 
		//this.findBoundary(groundEndPointIdx, angleGroundPointThres);
		//modify the boundary using the ground plane information, vectors with vertical angle to the plane will be identified as boundary
		//this.modifyBoundary();
		//this.transfrom3DPoint();
		//make object height mask based on 3D points w.r.t ground plane centered by the lidar projection
		//this.makeObjHeightMask();
	}

	public void processLog() throws IOException{
		this.reset();
		
		this.logDistanceTable(new FileOutputStream(new File("/home/qichi/dist.log")));
		
		//calculate the angle between each vectors
		this.makeVertAngleTable();
		this.logVertAngleTable(new FileOutputStream(new File("/home/qichi/angle.log")));
		//find the first non-ground points
		this.findBoundary(groundFitIdx, angleGroundFitThres);
		
		//fit plane
		this.fitGroundPlane(this.vtable.getStartIdx(), this.groundFitIdx);
		
		this.findBoundary(groundEndPointIdx, angleGroundPointThres);
		this.logBoundary(new FileOutputStream(new File("/home/qichi/boundary.log")));
		//transform points from
		this.transfrom3DPoint();
		//make object height mask based on 3D points w.r.t ground plane centered by the lidar projection
		this.makeObjHeightMask();
	}
	
	public void reset(){
		this.vertAngleTable = new double[rows][cols];
		this.groundEndPointIdx = new int[this.vtable.getColNum()];
		this.groundFitIdx = new int[this.vtable.getColNum()];
		this.objHeightMask = new int[this.vtable.getRowNum()][cols];
		this.plane3DPoints = new Point3D[this.vtable.getRowNum()][cols];
	}
	
	//make a mask matrix based on point height w.r.t ground plane
	//0-ground, 1-low, 2-mid, 3-high
	private void makeObjHeightMask(){
		//default 0, means ground points
		int rows = this.vtable.getRowNum();
		int cols = this.vtable.getColNum();
		
		int[] start = this.groundEndPointIdx;
		objNum=new int[] {0,0,0,0};
		//col first
		for(int i=0; i<cols; i++){
			for(int j=start[i]; j<rows; j++){
				Point3D p = this.plane3DPoints[j][i];
				if(p!=null){
					if(p.z < objLow){
						this.objHeightMask[j][i] = 1;
						objNum[1]++;
					}else if(p.z > objHigh){
						this.objHeightMask[j][i] = 3;
						objNum[3]++;
					}else{
						this.objHeightMask[j][i] = 2;
						objNum[2]++;
					}
				}else{
					objNum[0]++;
				}
			}
		}
		//objNum[0]=totalNum - objNum[1] - objNum[2] -objNum[3];
	}
		
	//transform 3D points from lidar frame to ground plane frame
	private void transfrom3DPoint(){
		int rows = this.vtable.getRowNum();
		int cols = this.vtable.getColNum();
		
		int[] start = this.groundEndPointIdx;
		//col first
		for(int i=0; i<cols; i++){
			for(int j=start[i]; j<rows; j++){
				Point3D p = this.vtable.getPoint3D(j, i);
				if(p!=null){
					this.plane3DPoints[j][i] = this.gpa.lidar2Plane(p);
				}
			}
		}
	}
	
	//use GroundPlanePCA to fit a plane
	//feed the ground points within localPlaneRange
	private void fitGroundPlane(int[] startColIdx, int[] endColIdx){
		int cols = this.vtable.getColNum();
		
		groundPoints.clear();
		//col first
		for(int i=0; i<cols; i++){
			for(int j=startColIdx[i]; j<endColIdx[i]; j++){
				Point3D p = this.vtable.getPoint3D(j, i);
				if(p!=null &&
					p.x<=localPlaneXrange && p.x>=-localPlaneXrange &&
					p.y<=localPlaneYrange && p.y>=-localPlaneYrange)
					groundPoints.add(p);
			}
		}
		gpa.addData(groundPoints);
		gpa.computeBasis();
		//gpa.printPlaneVectors();
	}
	
	private void modifyBoundary(){
		int[] startIdx = this.vtable.getStartIdx();
		Point3D p0, p1;
		//System.out.println("modifyBoundary");
		for(int i=0; i<this.cols; i++){
			if(this.groundEndPointIdx[i]==0) continue;
			int j=startIdx[i];
			if(j>=this.vtable.getRowNum()) continue;
			p0 = this.vtable.getPoint3D(j, i);
			while(++j<this.vtable.getRowNum()){
				if(this.vtable.getDistance(j, i)!=0){
					p1 = this.vtable.getPoint3D(j, i);
					if(this.gpa.calcVectorAngle(p1.minus(p0))<angleObjThres){
						this.groundEndPointIdx[i] = startIdx[i];
						//System.out.printf("find vertical start slope at (%d, %d)\n", j, i);
						break;
					}
				}
			}
		}
	}
	
	//calculate the cos angle between each 3 points(2 vectors) and save the result in angle table
	private void makeVertAngleTable(){
		//calculate slope along each column
		for(int i=0; i<cols; i++){
			//find boundary row by row
			Point2D.Double p0=null, p1=null, p2=null;
			int row1=-1;
			int j=-1;
			
			//find first non-zero point
			while(++j<rows){
				if(this.vtable.getDistance(j, i)!=0) {
					p0 = this.vtable.getVerticalPoint(j, i); 
					break;
				}
			}
			//find second non-zero point
			while(++j<rows){
				if(this.vtable.getDistance(j, i)!=0){
					p1 = this.vtable.getVerticalPoint(j, i);
					row1=j;
					break;
				}					
			}
			//start calculating angle when meeting non-zero points
			while(++j<rows){
				if(this.vtable.getDistance(j, i)!=0){
					p2 = this.vtable.getVerticalPoint(j, i);
					this.vertAngleTable[row1-1][i] = this.calcVecAngleCos(p0, p1, p2);//from 0 to pi
					p0=p1; p1=p2; row1=j;
				}
			}
		}
	}
	
	//calculate cos of angle between vector p1p0 and p1p2
	//row, col - idx of p1
	private double calcVecAngleCos(Point2D.Double p0, Point2D.Double p1, Point2D.Double p2){
		Point2D.Double p1p0 = new Point2D.Double(p0.x-p1.x, p0.y-p1.y);
		Point2D.Double p1p2 = new Point2D.Double(p2.x-p1.x, p2.y-p1.y);

		return Math.acos((p1p0.x*p1p2.x + p1p0.y*p1p2.y) / (p0.distance(p1) * p2.distance(p1)));
	}

	//find the ground points in each column
	//then tag the idx of points which first goes below the threshold in each column as the boundary 
	private void findBoundary(int[] endIdxArray, double angleThres){
		//find ground boundary, put the idx of the first of points whose slope smaller than threshold
		for(int i=0; i<cols; i++){
			int j=1;
			int lastMeas=-1;
			for(; j<rows-1; j++){
				if(this.vertAngleTable[j-1][i]!=0){
					lastMeas=j;
					if(this.vertAngleTable[j-1][i]<angleThres){
						endIdxArray[i]=j+1;//the idx of pt after the last ground point
						break;
					}
				}
			}
			//if the no angle under threshold is detected, set idx to lastMeas+1 to indicate the whole column(up to the last non-zero row) is ground
			if(j==rows-1)
				endIdxArray[i] = lastMeas+1;
		}
	}
	
	public FloatBuffer getPlaneGridDataBuffer(){
		return this.gpa.getPlaneGrid();
	}
	
	public int getGridPointNum(){
		return this.gpa.getGridPointNum();
	}
	
	public void logGroundPlane(FileOutputStream fs, BodyFrame bodyFrame) throws IOException{
		CoordinateFrame gf = this.gpa.getGroundFrameInWorld(bodyFrame);
		fs.write(gf.toString().getBytes());
	}
	
	public void logGroundPlane(FileOutputStream fs) throws IOException{
		CoordinateFrame gf = this.gpa.getGroundFrameInLidar();
		fs.write(gf.toString().getBytes());
	}
	
	public void logGroundPoints(FileOutputStream fs) throws IOException{
		for(int i=0;i<groundPoints.size();i++){
			Point3D p = groundPoints.get(i);
			fs.write(String.format("%.5f,%.5f,%.5f\n", p.x, p.y, p.z).getBytes());
		}
		fs.close();
	}
	
	public FloatBuffer getGroundFitPointsBuffer(){
		FloatBuffer fb = Buffers.newDirectFloatBuffer(this.rows*this.cols*3);
		for(int i=0;i<groundPoints.size();i++){
			Point3D p = groundPoints.get(i);
			fb.put((float)p.x);fb.put((float)p.y);fb.put((float)p.z);
		}
		fb.rewind();
		return fb;
	}
	
	public int getPointNum(){
		return groundPoints.size();
	}
	
	public void logVertAngleTable(FileOutputStream fs) throws IOException{
		fs.write("slope\n".getBytes());
		fs.write(this.vtable.getAttribute().getBytes());
		fs.write('\n');
		
		int cols = this.vtable.getColNum();
		int rows = this.vtable.getRowNum()-2;
		
		//col first
		for(int i=0; i<cols; i++){
			for(int j=0; j<rows; j++){
				fs.write(String.format("%.2f,", this.vertAngleTable[j][i]).getBytes());
			}
			fs.write('\n');
		}
		fs.close();
	}
	
	public void logBoundary(FileOutputStream fs) throws IOException{
		int len = this.groundEndPointIdx.length;
		
		for(int i=0;i<len;i++){
			fs.write(String.format("%d",this.groundEndPointIdx[i]).getBytes());
			fs.write(',');
		}
		fs.close();
	}
	
	public void logDistanceTable(FileOutputStream fs) throws IOException{
		this.vtable.logDistanceTable(fs);	
	}
	
	public void logMask3DPlanePointWithIntensity(DataOutputStream dos, int mask) throws IOException{
		this.logMaskPoint(dos, mask, this.plane3DPoints);
	}
	
	public void logMask3DLidarPointWithIntensity(DataOutputStream dos, int mask) throws IOException{
		this.logMaskPoint(dos, mask, this.vtable.getLidarPoint3D());
	}
	
	private void logMaskPoint(DataOutputStream dos, int mask, Point3D[][] points) throws IOException{
		dos.writeInt(this.objNum[mask]); System.out.println(this.objNum[mask]);
		
		int rows = this.vtable.getRowNum();
		int cols = this.vtable.getColNum();
		
		int[] start = this.groundEndPointIdx;
		//col first
		for(int i=0; i<cols; i++){
			for(int j=start[i]; j<rows; j++){
				Point3D p = points[j][i];
				float intensity = (float)this.vtable.getIntensity(j,i);
				if (p!=null && this.objHeightMask[j][i]==mask && intensity!=VirtualTable.defaultNullIntensity){
					dos.writeFloat((float)p.x);dos.writeFloat((float)p.y);dos.writeFloat((float)p.z);
					dos.writeFloat(intensity);
				}
			}
		}
	}
	
	//log raw data in Lidar frame with intensity
	private void logRawData(DataOutputStream dos, LidarFrame lf, double z_threshold) throws IOException{
		Point3D[] new_points = new Point3D[lf.getPointNum()];
		int cnt=0;
		for(int i=0; i<lf.getPointNum(); i++){
			Point3D p = lf.getDataPoint(i);
			Point3D new_p = this.gpa.lidar2Plane(p);
			if(new_p.z<z_threshold){
				new_points[i]=p;//to Lidar frame
				//new_points[i]=new_p;//to Plane frame
				cnt++;
			}
		}
		dos.writeInt(cnt);
		for(int i=0;i<lf.getPointNum(); i++){
			Point3D p = new_points[i];
			if(p!=null){
				dos.writeFloat((float)p.x);dos.writeFloat((float)p.y);dos.writeFloat((float)p.z);
				dos.writeFloat(lf.getIntensity(i));
			}
		}
	}
}

