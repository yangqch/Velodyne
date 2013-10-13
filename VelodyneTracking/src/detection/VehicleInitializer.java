package detection;

import java.util.ArrayList;

import velodyne2d.FrameTransformer2D;
import velodyne2d.Line;
import velodyne2d.Point2D;
import velodyne2d.Segment;
import velodyne2d.Vector;


/**
 * initialize vehicle from segment and its displacement
 * segment will be filtered by
 * 1. numOfPoint
 * 2. convexity: dist_thres(a line is constructed from first and last points, distance of points larger than dist_thres should fall in one side of the line)
 * 3. numOfLines: break end-point method
 * 4. length and angle of each line and between 2 lines
 * 
 * Vehicle will be initialzed by
 * 1. angle of the line -> orientation
 * 2. length of each line -> size
 * 3. displacement -> speed
 * 
 * @author qichi
 *
 */
public class VehicleInitializer {
	static double maxLineAngle=15;//-/+5 degree from 90 degree
	static double defaultVehWidth=2;
	static double defaultVehLength=3;
	
	//private LineExtractor lineExt;
	private Line[] lines;
	public int numOfLine;
	private int frontLineIdx;
	private int pointStartIdx;
	private int breakPointIdx;
	private int pointEndIdx;
	public Point2D corner;
	private Vector normal;//normal vector of the front line
	
	//tools
	private PCALineFitter pcaLineFitter;
	private FrameTransformer2D trans;
	
	public VehicleInitializer() {
		trans = new FrameTransformer2D();
		pcaLineFitter = new PCALineFitter();
	}
	
	public VehicleModel initialize(Segment seg){
		this.numOfLine = 0;
		
		if(seg.getMotion().isMoving()==0) return null;//only initialize moving segments
		
		lines = new Line[2];//at most 2 lines

		Line[] rawLines = seg.getLineExtractor().getLines();
		if(rawLines.length==0){
			System.out.println("Vehicle Initializer: reject because no line is extracted");
			return null;
		}
		if(rawLines.length>2){
			System.out.printf("Vehicle Initializer: reject because more than two lines, %d lines\n", rawLines.length);
			return null;
		}
		if(rawLines.length==1){//send to initialize
			this.lines[0]=new Line(rawLines[0]);
			this.numOfLine=1;
			this.frontLineIdx=0;
			this.pointStartIdx = 0;
			this.pointEndIdx = seg.getNumOfPts()-1;
			this.breakPointIdx = -1;
		}else{//check angle between two lines
			if(!checkLineAngle(rawLines)){
				return null;
			}
			//if one line has only two points, throw away
			this.breakPointIdx = seg.getLineExtractor().getBreakPointIdx(1);
			if(seg.getNumOfPts()-this.breakPointIdx <= 2){
				this.lines[0]=new Line(rawLines[0]);
				this.numOfLine=1;
				this.frontLineIdx=0;
				this.pointStartIdx = 0;
				this.pointEndIdx = this.breakPointIdx;
			}else if(this.breakPointIdx<2){
				this.lines[0]=new Line(rawLines[1]);
				this.numOfLine=1;
				this.frontLineIdx=0;
				this.pointStartIdx = this.breakPointIdx;
				this.pointEndIdx = seg.getNumOfPts()-1;
			}else{//if two lines are both valid, find front line(more motion points), so the orientation of the vehicle can be determined
				this.lines[0]=new Line(rawLines[0]);
				this.lines[1]=new Line(rawLines[1]);
				this.numOfLine=2;
				this.pointStartIdx = 0;
				this.pointEndIdx = seg.getNumOfPts()-1;
				this.frontLineIdx=this.findFrontLine(seg);//after this line, this.lines should contain two lines	
			}
		}
		//adjust the angle of lines, so they can be perpendicular to each other
		this.adjustLines2(seg);
		this.corner = this.numOfLine==2 ? this.lines[this.frontLineIdx].p1 : this.lines[this.frontLineIdx].getMidPoint();
		//find normal vector based on front line
		this.normal = this.findNormalVector(this.lines[this.frontLineIdx], seg.getMotion().isMoving());
		
		//System.out.printf("find detection candidate: %d lines, corner at %s, motion %d, normal vector %s\n", numOfLine, this.corner, seg.getMotion().isMoving(), this.normal);
		
		return makeVehicle(seg);
	}
	
	private boolean checkLineAngle(Line[] lines){
		if(lines.length!=2){
			System.err.printf("Vehicle Initializer: checkLineAngle() only check two lines, %d is passed in\n", lines.length);
			return false;
		}
		double angleDiff = Math.abs(lines[0].getAngleDeg() - lines[1].getAngleDeg());
		if(Math.abs(angleDiff-90.0)<=maxLineAngle){
			return true;
		}else{
			System.out.printf("Vehicle Initializer: reject because angle between two lines, %.3f\n", angleDiff);
			return false;
		}
	}
	
	/**
	 * adjust the one or two lines
	 * so that the distance of data points on each part to each line can be minimized.
	 * @param seg
	 */
	private void adjustLines2(Segment seg){
		if(this.numOfLine==1){
			Line line = this.pcaLineFitter.fitOneLinePCA(seg.getPoints(), this.pointStartIdx, this.pointEndIdx+1);
			this.lines[0] = line;
		}else{
			Line[] lines = this.pcaLineFitter.fitTwoLinePCA(seg.getPoints(), this.breakPointIdx);
			this.lines[0] = lines[0];
			this.lines[1] = lines[1];
		}
		
	}
	
	/**
	 * check the average distance between points to each line
	 * the line with min distance is more trustable
	 * adjust the other line to be perpendicular to this one
	 * determine Point2D corner which is the intersection of two lines
	 * @param lines
	 */
	private void adjustLines(Segment seg){
		if(this.lines.length!=2){
			System.err.printf("Vehicle Initializer: adjustLines() only check two lines, %d is passed in\n", this.lines.length);
		}
		Point2D[] points = seg.getPoints();
		int breakIdx = seg.getLineExtractor().getBreakPointIdx(1);
		double dist0=0, dist1=0;
		//test line 0
		for(int i=0; i<=breakIdx; i++){
			//calculate the distance from point to line
			double d = this.lines[0].calcDist(points[i]);
			dist0 += d;
		}
		dist0/=breakIdx;
		//test line 1
		for(int i=breakIdx; i<points.length; i++){
			//calculate the distance from point to line
			double d = this.lines[1].calcDist(points[i]);
			dist1 += d;
		}
		dist1/=points.length-breakIdx+1;
		
		//determine trustable line based on the distance
		
		Point2D p1 = points[0];
		Point2D p2 = points[breakIdx];
		Vector p1p2 = new Vector(p1, p2);
		Point2D p3 = points[points.length-1];
		Vector p3p2 = new Vector(p3, p2);
		if(dist0 <= dist1){//trust line0
			double alpha = (p1.x*p1p2.x - p3.x*p1p2.x + p1.y*p1p2.y - p3.y*p1p2.y)/(-p1p2.normSq());
			this.corner = new Point2D(p1.x + alpha*p1p2.x, p1.y + alpha*p1p2.y);
		}
		else if(dist1 < dist0){//trust line1
			double alpha = (p3.x*p3p2.x - p1.x*p3p2.x + p3.y*p3p2.y - p1.y*p3p2.y)/(-p3p2.normSq());
			this.corner = new Point2D(p3.x + alpha*p3p2.x, p3.y + alpha*p3p2.y);
		}
		this.lines[0] = new Line(this.corner, p1);
		this.lines[1] = new Line(this.corner, p3);
	}
	
	/**
	 * find major line if 2 lines are extracted from a segment
	 * the major line is the line with most motion points
	 * @param lines
	 * @param seg
	 */
	private int findFrontLine(Segment seg){
		if(this.lines.length!=2){
			System.err.printf("Vehicle Initializer: findMajorLine() only check two lines, %d is passed in\n", this.lines.length);
		}
		int breakIdx = seg.getLineExtractor().getBreakPointIdx(1);//assume there are only two lines, so the second breakpoint is the common point shared by the two lines
		//this.corner = seg.getPoints()[breakIdx];
		
		Motion motion = seg.getMotion();
//		int totalPoints = seg.getNumOfPts();
//		double numOfPts1 = breakIdx, numOfPts2 = totalPoints-breakIdx;
//		
//		int numOfMotion1=0, numOfMotion2=0;
//		for(int i=0; i<=breakIdx; i++){
//			if(motion.prevMove[i]>0 && motion.isMoving()>0) numOfMotion1++;
//			else if(motion.prevMove[i]<0 && motion.isMoving()<0) numOfMotion1++;
//		}
//		for(int i=breakIdx; i<totalPoints; i++){
//			if(motion.prevMove[i]>0 && motion.isMoving()>0) numOfMotion2++;
//			else if(motion.prevMove[i]<0 && motion.isMoving()<0) numOfMotion2++;
//		}
//		if(numOfMotion1/numOfPts1 > numOfMotion2/numOfPts2){
//			majorLineIdx=0;
//			this.motionPtIdx1=0; this.motionPtIdx2=breakIdx;
//		}else{
//			majorLineIdx=1;
//			this.motionPtIdx1=breakIdx; this.motionPtIdx2=seg.getNumOfPts()-1;
//		}
		
		if(motion.isMoving()>0){
			return this.findMostMotion(motion.prevMove, breakIdx);
		}else{
			return this.findMostMotion(motion.nextMove, breakIdx);
		}
	}
	/**
	 * find which part in a point array(2 parts) contain most number of right motion
	 * always consider the motionDist>0
	 * because when getting further, we pass in the future motion
	 * and when getting closer, we pass in the previous motion 
	 * @param motionDist, a double array of motion distance stored in motion
	 * @param breakIdx, idx of break point
	 * @return
	 */
	private int findMostMotion(double[] motionDist, int breakIdx){
		float numOfMotion1=0, numOfMotion2=0;
		float numOfPts1 = breakIdx, numOfPts2 = motionDist.length-breakIdx;
		for(int i=0; i<=breakIdx; i++){
			if(motionDist[i]>0) numOfMotion1++;
		}
		for(int i=breakIdx; i<motionDist.length; i++){
			if(motionDist[i]>0) numOfMotion2++;
		}
		return numOfMotion1/numOfPts1 > numOfMotion2/numOfPts2 ? 0 : 1;
	}
	
	
	/**
	 * find normal vector(direction) of the car
	 * normal vector is perpendicular to front line
	 * its direction depends on motion(the car is moving away to closer)
	 * @param front
	 * @param motion
	 * @return
	 */
	private Vector findNormalVector(Line front, int motion){
		double initAngle = front.getPerpendicularAngleRad();//in radius
		Vector initNormal = new Vector(Math.cos(initAngle), Math.sin(initAngle));
		Vector p1o = new Vector(-front.p1.x, -front.p1.y);
		double dot = initNormal.dot(p1o);
		if(motion>0 && dot>0){
			initNormal.x = -initNormal.x;
			initNormal.y = -initNormal.y;
		}else if(motion<0 && dot<0){
			initNormal.x = -initNormal.x;
			initNormal.y = -initNormal.y;
		}
		initNormal.normalize();
		return initNormal;
	}
	
	/**
	 * calculate the initial speed of the vehicle 
	 * average motion displacement along normal vector
	 * @return
	 */
	private double calcSpeed(Motion motion){//fix, motionIdx
		int motionPtIdx1=-1, motionPtIdx2=-1;
		if(this.numOfLine==1){
			motionPtIdx1 = this.pointStartIdx;
			motionPtIdx2 = this.pointEndIdx;
		}else if(this.numOfLine==2){
			if(this.frontLineIdx==0){
				motionPtIdx1 = this.pointStartIdx;
				motionPtIdx2 = this.breakPointIdx;
			}else if(this.frontLineIdx==1){
				motionPtIdx1 = this.breakPointIdx;
				motionPtIdx2 = this.pointEndIdx;
			}else{
				throw new RuntimeException(String.format("Vehicle Initializer: calcuSpeed only work for 2 lines when front line is detected, it is now %d\n", this.frontLineIdx));
			}
			
		}else{
			throw new RuntimeException(String.format("Vehicle Initializer: calcuSpeed only work when 1 or 2 lines are available, it is now %d\n", this.numOfLine));
		}
		
		//when getting further, use previous motion
		try{
			if(motion.isMoving()>0){
				return this.findMaxMontion(motion.prevMove, motion.prevMoveVector, motionPtIdx1, motionPtIdx2);
			}else{//when getting closer, use future motion
				return this.findMaxMontion(motion.nextMove, motion.nextMoveVector, motionPtIdx1, motionPtIdx2);
			}
		}catch(Exception e){
			System.out.println("");
			throw new RuntimeException();
		}
		
	}
	
	/**
	 * find the max speed from motionDist with largest positive motion along normal vector
	 * @param motionDist
	 * @param start
	 * @param end
	 * @return
	 */
	private double findMaxMontion(double[] motionDist, Vector[] motionVector, int start, int end){
		double maxDist=-1; 
		int idx=-1;
		for(int i=start; i<=end; i++){
			if(motionDist[i]<=0){
				continue;
			}
			if(motionDist[i]>maxDist){
				maxDist = motionDist[i];
				idx = i;
			}
		}
		double speed = VehicleModel.default_speed;
		if(idx!=-1){
			speed = this.normal.dot(motionVector[idx])*motionDist[idx];
		}
		return Math.abs(speed);
	}
	
	/**
	 * create Vehicle using two lines
	 * @return
	 */
	private VehicleModel makeVehicle(Segment seg){
		Line front = this.getFrontLine();
		//width, length
		double width = this.getFrontLine().length < VehicleModel.default_width ? VehicleModel.default_width : this.getFrontLine().length;
		double length = this.numOfLine==2 ? (this.getSideLine().length < VehicleModel.default_length ? VehicleModel.default_length : this.getSideLine().length) : VehicleModel.default_length;
		//orientation = this.normalVector
		
		//speed(motion displacement along orientation, motionPtIdx1, motionPtIdx2)
		double speed = this.calcSpeed(seg.getMotion());
		//anchor point(different for one line or two lines)
		
		//center
		Vector frontVec = new Vector(front.p1, front.p2);
		frontVec.normalize();
		Vector sideVec = seg.getMotion().isMoving()>0 ? new Vector(this.normal.x, this.normal.y) : new Vector(-this.normal.x, -this.normal.y);
		sideVec.normalize();
		Point2D center = null;
		if(this.numOfLine==1){
			Point2D start = front.getMidPoint();
			center = new Point2D(start.x + sideVec.x * length/2, start.y + sideVec.y * length/2);
		}
		//from corner, move width/2 along front line
		//move length/2 along inverse normal vector
		else if(this.numOfLine==2){
			Point2D start = this.corner;
			center = new Point2D(start.x + frontVec.x * width/2 + sideVec.x * length/2, start.y + frontVec.y * width/2 + sideVec.y * length/2);
		}else{
			return null;
		}
		
		Point2D w_center = trans.transform(seg.getFrame(), null, center);
		
		return new VehicleModel(w_center, this.normal, width, length, speed);
	}
	
	public Line getFrontLine(){
		return this.lines[this.frontLineIdx];
	}
	
	public Line getSideLine(){
		return this.lines[this.frontLineIdx==0 ? 1 : 0];
	}
	
	public Line getNormalVectorLine(){
		return new Line(this.corner, new Point2D(this.corner.x+this.normal.x*3.0, this.corner.y+this.normal.y*3.0));
	}
	
	public Line[] getLines(){
		return lines;
	}
}
