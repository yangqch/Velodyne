package detection;

import velodyne2d.FrameTransformer2D;
import velodyne2d.Line;
import velodyne2d.Point2D;
import velodyne2d.Polar;
import velodyne2d.Segment;
import velodyne2d.Vector;
import velodyne2d.VirtualScan;

public class Motion {
	//threshold
	static double minMotionDist=0.2;
	static double maxMotionDist=2.0;
//	static double motionPtsRatio=0.5;
	static double minNumOfMotion=3;
	//
	private int numOfPts;
	double[] prevMove;//motion w.r.t previous scan
	Vector[] prevMoveVector;//the norm vector of motion which the ray is on in previous frame
	double[] nextMove;//motion w.r.t next scan
	Vector[] nextMoveVector;//the norm vector of motion which the ray is on in previous frame
	private Segment segment;
	private FrameTransformer2D trans;
	//
	private boolean motionDetected;
	private int motion;//0:stationary, -1:closer, 1:further
	
	public Motion(Segment seg) {
		segment = seg;
		segment.setMotion(this);
		numOfPts = segment.getNumOfPts();
		trans = new FrameTransformer2D();
		motion=0;
		motionDetected=false;
	}
	
	public void detectMotionFromPrev(VirtualScan prevScan){
//		System.out.printf("segment %d detect motion prev\n", segment.getId());
		prevMove = new double[numOfPts];
		prevMoveVector = new Vector[numOfPts];
		this.detectMotion(prevScan, prevMove, prevMoveVector);	
	}
	
	public void detectMotionFromNext(VirtualScan nextScan){
//		System.out.printf("segment %d detect motion next\n", segment.getId());
		nextMove = new double[numOfPts];
		nextMoveVector = new Vector[numOfPts];
		this.detectMotion(nextScan, nextMove, nextMoveVector);	
	}
	
	/**
	 * detect motion from segment.mFrame(curFrame) to newScan
	 * motion = ray.dist(curFrame) - ray.dist(newFrame)
	 * store motion in motion
	 * @param newScan
	 * @param motion
	 */
	private void detectMotion(VirtualScan newScan, double[] motion, Vector[] motionVector){
		Point2D[] segPointsInNew = trans.transform(this.segment.getFrame(), newScan.getLocalWorldFrame(), this.segment.getPoints());
		//compare the position of each point along each ray in previous frame
		int i=0;
		for(Point2D p: segPointsInNew){
			Polar polar = newScan.XY2polar(p);
			int idx = newScan.angle2Index(polar.angle);
			double newDist = newScan.getDistance(idx);
			double distDiff = polar.dist - newDist;
			//System.out.printf("angle %.1f(index %d), dist %.1f, diff %.1f\n", polar.angle, idx, polar.dist, distDiff);
			if(Math.abs(distDiff)>=minMotionDist && Math.abs(distDiff)<=maxMotionDist){
				motion[i]=distDiff;//distDiff>0, getting further; distDiff<0, getting closer
			}else{// if(Math.abs(distDiff)<=minMotionDist){
				motion[i]=0;//small displacement, no move 
			}
			//make motion vector, the point p in new coordinate frame
			Vector x = new Vector(p.x, p.y);
			x.normalize();
			motionVector[i] = x; 
			//
			i++;
		}
	}
	
	public int isMovingPrev(){
		int cnt=0;
		for(int ii=0; ii<numOfPts; ii++){
			if(prevMove[ii]!=0){
				cnt++;
			}
		}
		if(cnt>10){
			cnt=0;
		}
		
		int numFurther=0, numCloser=0;
		int seqFurther=0, seqCloser=0;
		int maxSeqFurther=0, maxSeqCloser=0;
		for(int i=0; i<numOfPts; i++){
			if(prevMove[i]==0){
				maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther; 
				seqFurther=0;
				maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
				seqCloser=0;
				continue;//no motion detected in either frame
			}
			if(Math.abs(prevMove[i])>maxMotionDist){
				maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther; 
				seqFurther=0;
				maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
				seqCloser=0;
				continue;//too big motion are ignored
			}
			if (prevMove[i]>0) numFurther++;
			if (prevMove[i]<0) numCloser++;
			
			if(prevMove[i]>0){
				seqFurther++;
			}else{
				maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther; 
				seqFurther=0;
			}
			if(prevMove[i]<0){
				seqCloser++;
			}else{
				maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
				seqCloser=0;
			}
		}
		maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther;
		maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
		//System.out.printf("prev: max seq further: %d, max seq closer: %d\n", maxSeqFurther, maxSeqCloser);
		
//		if(numFurther - numCloser >= minNumOfMotion) return 1;
//		else if(numCloser - numFurther >= minNumOfMotion) return -1;
//		else return 0;
		
//		if(numFurther>minNumOfMotion && numCloser==0) return 1;
//		else if(numCloser>minNumOfMotion && numFurther==0) return -1;
//		return 0;
		
		if(maxSeqFurther>minNumOfMotion && maxSeqFurther > maxSeqCloser) return 1;
		else if(maxSeqCloser>minNumOfMotion && maxSeqCloser > maxSeqFurther) return -1;
		return 0;
	}
	
	public int isMovingNext(){
		int numFurther=0, numCloser=0;
		int seqFurther=0, seqCloser=0;
		int maxSeqFurther=0, maxSeqCloser=0;
		for(int i=0; i<numOfPts; i++){
			if(nextMove[i]==0){
				maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther; 
				seqFurther=0;
				maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
				seqCloser=0;
				continue;//no motion detected in either frame
			}
			if(Math.abs(nextMove[i])>maxMotionDist){
				maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther; 
				seqFurther=0;
				maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
				seqCloser=0;
				continue;//too big motion are ignored
			}
			if (nextMove[i]<0) numFurther++;
			if (nextMove[i]>0) numCloser++;
			
			if(nextMove[i]<0){
				seqFurther++;
			}else{
				maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther; 
				seqFurther=0;
			}
			if(nextMove[i]>0){
				seqCloser++;
			}else{
				maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
				seqCloser=0;
			}
		}
		maxSeqFurther = seqFurther > maxSeqFurther ? seqFurther : maxSeqFurther;
		maxSeqCloser = seqCloser > maxSeqCloser ? seqCloser : maxSeqCloser;
		//System.out.printf("next: max seq further: %d, max seq closer: %d\n", maxSeqFurther, maxSeqCloser);
		
//		if(numFurther - numCloser >= minNumOfMotion) return 1;
//		else if(numCloser - numFurther >= minNumOfMotion) return -1;
//		else return 0;
		
//		if(numFurther>minNumOfMotion && numCloser==0) return 1;
//		else if(numCloser>minNumOfMotion && numFurther==0) return -1;
//		return 0;
		
		if(maxSeqFurther>minNumOfMotion && maxSeqFurther > maxSeqCloser) return 1;
		else if(maxSeqCloser>minNumOfMotion && maxSeqCloser > maxSeqFurther) return -1;
		return 0;
	}
	
	public int isMoving(){
		if(!motionDetected){
			int prev=this.isMovingPrev();
			int next=this.isMovingNext();
			if(prev<0 && next<0) motion = -1;
			else if(prev>0 && next>0) motion = 1;
			else motion =0;
		}
		return motion;
	}
	
	/**
	 * if points have consistent moving in prevMove and nextMove, determine the motion
	 * otherwise, consider stationary
	 * @return
	 */
	public int isMovingCombine(){
		//if(motionDetected) return motion;
		
		int numFurther=0, numCloser=0;
		for(int i=0; i<numOfPts; i++){
			if(prevMove[i]==0 || nextMove[i]==0){
				continue;//no motion detected in either frame
			}
			if(Math.abs(prevMove[i])>maxMotionDist && Math.abs(nextMove[i])>maxMotionDist){
				continue;//too big motion are ignored
			}
			if(Math.abs(prevMove[i])<=maxMotionDist && Math.abs(nextMove[i])<=maxMotionDist){
				if((prevMove[i]<0 && nextMove[i]>0) || (prevMove[i]>0 && nextMove[i]<0)){
					continue;//motion not agree
				}else{
					if (prevMove[i]>0) numFurther++;
					if (prevMove[i]<0) numCloser++;
				}
			}else{//some one is igorned
				if((Math.abs(prevMove[i])>maxMotionDist && nextMove[i]<0)
						|| (Math.abs(nextMove[i])>maxMotionDist && prevMove[i]>0)){
					numFurther++;
				}else{
					numCloser++;
				}
			}
		}
		if(numFurther - numCloser > minNumOfMotion) motion = 1;
		else if(numCloser - numFurther > minNumOfMotion) motion = -1;
		else motion = 0;
		
		//motionDetected=true;
		
		System.out.printf("further: %d, closer: %d\n", numFurther, numCloser);
		
		return motion;
	}
	
	public Line[] getPrevMoveLines(Point2D[] points){
		Line[] lines = new Line[points.length];
		for(int i=0; i<points.length; i++){
			Point2D p = new Point2D(points[i].x - prevMoveVector[i].x*prevMove[i], points[i].y - prevMoveVector[i].y*prevMove[i]);
			lines[i] = new Line(points[i], p); 
		}
		return lines;
	}
	
	public Line[] getNextMoveLines(Point2D[] points){
		Line[] lines = new Line[points.length];
		for(int i=0; i<points.length; i++){
			Point2D p = new Point2D(points[i].x - nextMoveVector[i].x*nextMove[i], points[i].y - nextMoveVector[i].y*nextMove[i]);
			lines[i] = new Line(points[i], p); 
		}
		return lines;
	}
}
