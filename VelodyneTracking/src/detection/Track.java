package detection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;

import VelodyneDataIO.LidarFrame;

import velodyne2d.FrameTransformer2D;
import velodyne2d.Point2D;
import velodyne2d.Vector;
import velodyne2d.VirtualScan;

public class Track {
//	static int numOfParticles=500;
//	static double lowProb=0.01;
//	static double angleRateLowSpeed=20;
//	static double angleRateHighSpeed=15;
//	static double acceleratePcrt=0.1;
//	static double minSpeed=0.1;
//	static double minAcc=0.2;
	public static double DEG2RAD=Math.PI/180.0;
	public static double RAD2DEG=180.0/Math.PI;
//	static double c_occl = 50;
//	static double c_bound = 100;
//	static double c_occupy = 0;
//	static double c_free = 100;
//	
//	static double s_occl = 0;
//	static double s_bound = -1;
//	static double s_occupy = 1;
//	static double s_free = -1;
//	
//	static double d_occupy = 0.3;
//	static double d_bound = 1.5;
//	static double scoreLimit = 100;
	
	static double likelihoodThres = 1;
	//////////////////////////////
	Particle[] particles;
	Particle[] resampleBuffer;//store resampled particles, swap after resampling each time
	VehicleModel origin;
	
	double[] cdf;
	Point2D[] centers;//vehicle centers, for particle visualize
	VehicleModel avgVehicle;//vehicle with average state
	
	int numOfLost;//num of lost before the track lost
	boolean lost;
	
	private ParticleComp comparator;
	private Random rand;
	
	private TrackConf params;
	private int numOfParticles;
	
	public Track(VehicleModel veh, TrackConf trackConf){
		params = trackConf;
		
		this.numOfParticles = params.numOfParticles;
		origin = veh;
		particles = new Particle[numOfParticles];
		resampleBuffer = new Particle[numOfParticles];
		cdf = new double[numOfParticles];
		centers = new Point2D[numOfParticles];
		
		for(int i=0; i<numOfParticles; i++){
			particles[i]=new Particle(veh.copy(), 1.0/numOfParticles);
			centers[i] = particles[i].vehicle.center;
			resampleBuffer[i] = particles[i];
		}
		rand = new Random();
		comparator = new ParticleComp();
		numOfLost=0;
		lost=false;
		
		this.initialize();
	}
	/**
	 * search around original vehicle to find a better prior distribution
	 */
	public void initialize(){
		this.interence();
	}
	
	public void diffuse(){
		for(Particle p : particles){
			VehicleModel vehicle = p.vehicle;
			double r = rand.nextDouble();
			double angleRateRange = vehicle.speed<params.minSpeed ? params.angleRateLowSpeed : params.angleRateHighSpeed;
			r = (r-0.5)*2;//to [-1, 1)
			double angle1 = angleRateRange * r * DEG2RAD;
			r = rand.nextDouble();
			r = (r-0.5)*2;//to [-1, 1)
			double angle2 = angleRateRange * r * DEG2RAD;
			r = rand.nextDouble();
			r = (r-0.5)*2;//to [-1, 1)
			double acc = (params.minAcc + vehicle.speed*params.acceleratePcrt) * r;
			
			vehicle.predict(angle1, acc, angle2);
		}
		for(int i=0; i<numOfParticles; i++){
			centers[i] = particles[i].vehicle.center;
		}
	}
	
	public void update(VirtualScan scan, FrameTransformer2D trans, boolean[] mask){
		this.update3(scan, trans, mask);
	}
	
//	public void update1(VirtualScan scan, FrameTransformer2D trans, boolean[] mask){
//		int maxHit=0;
//		Point2D local_orgin = new Point2D(0 ,0);//lidar origin in local world frame
//		Point2D body_origin=null;
//		for(int i=0; i<numOfParticles; i++){
//			Particle p = particles[i];
//			VehicleModel vehicle = p.vehicle;
//			
//			//check if the particle accidentally involve the lidar origin(it can cause bad problems)
//			body_origin = trans.transform(scan.getLocalWorldFrame(), vehicle.bodyFrame, local_orgin);
//			if(Math.abs(body_origin.x)<vehicle.shape.length/2 && Math.abs(body_origin.y)<vehicle.shape.width/2){
//				p.weight = 0; continue;
//			}
//			//score on the scan
//			vehicle.predictMeasurement(scan, trans);
//			ArrayList<RayMeas> rayMeas = vehicle.getRayMeas();
//			double cost = 0;
//			int numOfHit=0;
//			for(int j=0; j<rayMeas.size(); j++){
//				double real = scan.getDistance(rayMeas.get(j).idx);
//				if(real<=0){//blocked ray
//					cost += params.c_occl;
//				}else if(real>=LidarFrame.MAX_RANGE){//ray beyond max range
//					cost += params.c_free;
//				}
//				else{//other valid rays
//					double diff = rayMeas.get(j).distance - real;
//					if(Math.abs(diff)<params.d_occupy){
//						//avoid the masked points(usually set for road infra)
//						if(mask!=null && mask[rayMeas.get(j).idx]==false){
//							cost += params.c_bound;
//							continue;
//						}
//						cost += params.c_occupy; numOfHit++; 
//					}else if(diff<0){
//						
//					}
//					
//					else if(diff<=params.d_bound && diff>params.d_occupy){
//						cost += params.c_bound;
//					}else if(diff>params.d_bound){
//						cost += params.c_occl;
//					}else{
//						cost += params.c_free;
//					}
//				}
//			}
//			maxHit = maxHit<numOfHit ? numOfHit : maxHit;
//
//			cost/=rayMeas.size();
//			p.weight = Math.exp(-cost/100);
//
//			//debug
//			//if(numOfHit>0) System.out.printf("particle %d has hits %d, and cost %f, weight %f\n", i, numOfHit, cost, p.weight);
//		}
//
//		//check the scores for lost
//		if(maxHit<5){
//			numOfLost++;
//			lost = true;
//		}else{
//			numOfLost=0;
//			lost = false;
//		}
//	}
	
	public void update3(VirtualScan scan, FrameTransformer2D trans, boolean[] mask){
		int maxHit=0;
		Point2D local_orgin = new Point2D(0 ,0);//lidar origin in local world frame
		Point2D body_origin=null;
		for(int i=0; i<numOfParticles; i++){
			Particle p = particles[i];
			VehicleModel vehicle = p.vehicle;
			
			//check if the particle accidentally involve the lidar origin(it can cause bad problems)
			body_origin = trans.transform(scan.getLocalWorldFrame(), vehicle.bodyFrame, local_orgin);
			if(Math.abs(body_origin.x)<vehicle.shape.length/2 && Math.abs(body_origin.y)<vehicle.shape.width/2){
				p.weight = 0; continue;
			}
			//score on the scan
			vehicle.predictMeasurement(scan, trans);
			ArrayList<RayMeas> rayMeas = vehicle.getRayMeas();
			double score = 0;
			int numOfHit=0;
			for(int j=0; j<rayMeas.size(); j++){
				double real = scan.getDistance(rayMeas.get(j).idx);
				double diff = rayMeas.get(j).distance - real;
				if(real<=0){//blocked ray
					score += params.s_occl;
				}else if(real>=LidarFrame.MAX_RANGE){//ray beyond max range
					score += params.s_free;
				}else if(mask!=null && mask[rayMeas.get(j).idx]==false){
					//avoid the masked points(usually set for road infra)
					score += params.s_bound;
				}else if(Math.abs(diff)<params.d_occupy){
					score += params.s_occupy; numOfHit++;
				}else if(diff<0){
					score += params.s_free;
				}else if(diff<params.d_bound){
					score += params.s_bound;
				}else{
					score += params.s_occl;
				}
			}
			maxHit = maxHit<numOfHit ? numOfHit : maxHit;

//			cost/=rayMeas.size();
			p.weight = Math.exp(score);

			//debug
			//if(numOfHit>0) System.out.printf("particle %d has hits %d, and cost %f, weight %f\n", i, numOfHit, cost, p.weight);
		}

		//check the scores for lost
		if(maxHit<5){
			numOfLost++;
			lost = true;
		}else{
			numOfLost=0;
			lost = false;
		}
	}
	
//	public void update2(VirtualScan scan, FrameTransformer2D trans, boolean[] mask){
//		int maxHit=0;
//		Point2D local_orgin = new Point2D(0 ,0);//lidar origin in local world frame
//		Point2D body_origin=null;
//		for(int i=0; i<numOfParticles; i++){
//			Particle p = particles[i];
//			VehicleModel vehicle = p.vehicle;
//			
//			//check if the particle accidentally involve the lidar origin(it can cause bad problems)
//			body_origin = trans.transform(scan.getLocalWorldFrame(), vehicle.bodyFrame, local_orgin);
//			if(Math.abs(body_origin.x)<vehicle.shape.length/2 && Math.abs(body_origin.y)<vehicle.shape.width/2){
//				p.weight = 0; continue;
//			}
//			//score on the scan
//			vehicle.predictMeasurement(scan, trans);
//			ArrayList<RayMeas> rayMeas = vehicle.getRayMeas();
//			double cost = 0;
//			int numOfHit=0;
//			for(int j=0; j<rayMeas.size(); j++){
//				double real = scan.getDistance(rayMeas.get(j).idx);
//				if(real<=0){//blocked ray
//					cost += c_occl;
//				}else if(real>=LidarFrame.MAX_RANGE){//ray beyond max range
//					cost += c_free;
//				}else if(mask!=null && mask[rayMeas.get(j).idx]==false){//avoid the masked points(usually set for road infra)
//						cost += c_bound;
//						continue; 
//				}else{
//					rayMeas.get(j).real = scan.getPoint2D(rayMeas.get(j).idx);
//				}
//			}
//			for(RayMeas ray: rayMeas){//calculate scores
//				Point2D measPoint = trans.transform(scan.getLocalWorldFrame(), vehicle.getBodyFrame(), ray.real);
//			}
//			
//			
//			maxHit = maxHit<numOfHit ? numOfHit : maxHit;
//
//			cost/=rayMeas.size();
//			p.weight = Math.exp(-cost/100);
//
//			//debug
//			//if(numOfHit>0) System.out.printf("particle %d has hits %d, and cost %f, weight %f\n", i, numOfHit, cost, p.weight);
//		}
//
//		//check the scores for lost
//		if(maxHit<5){
//			numOfLost++;
//			lost = true;
//		}else{
//			numOfLost=0;
//			lost = false;
//		}
//	}
	
	/**
	 * normalize
	 */
	private void normalize(){
//		for(int i=0; i<numOfParticles; i++){
//			System.out.printf("%.3f,", particles[i].weight);
//			if(i%10==0) System.out.println();
//		}
//		System.out.println("======================");
		
		double sum=0;
		for(int i=0; i<numOfParticles; i++){
			sum+=particles[i].weight;
		}
		if(sum==0){
			for(int i=0; i<numOfParticles; i++){
				particles[i].weight=1.0/numOfParticles;
			}
		}else{
			for(int i=0; i<numOfParticles; i++){
				particles[i].weight/=sum;
			}	
		}
		
		
//		for(int i=0; i<numOfParticles; i++){
//			System.out.printf("(%d, %.3f),", i, particles[i].weight);
//			if(i%10==0) System.out.println();
//		}
//		System.out.println("======================");
	}
	/**
	 * use stochastic universal sampling
	 * if lost, no resampling, but inference
	 * o.w. kick out really bad samples and resampling, then inference
	 */
	public void resample(){
//		this.pirntPosterior();
		if(this.lost){//if lost, reset the weight
			//System.out.println("lost, skip resampling");
			for(int i=0; i<numOfParticles; i++){
				particles[i].weight=1.0/numOfParticles;
				resampleBuffer[i] = particles[i];
			}
			Particle[] swap = particles;
			particles = resampleBuffer;
			resampleBuffer = swap;
			this.interence();
			return;
		}else{
			this.normalize();
			Arrays.sort(particles, comparator);
			
//			for(int i=0; i<numOfParticles; i++){
//				if(particles[i].weight<likelihoodThres/numOfParticles){
//					particles[i].weight=0;
//				}
//			}
////			//only use first 1% to resample
////			for(int i=(int)Math.floor(this.numOfParticles*0.1); i<numOfParticles; i++){
////				particles[i].weight=0;
////			}
//			this.normalize();
//			//inference
			
			this.interence();
		}
//		this.pirntPosterior();
		
		cdf[0] = particles[0].weight;
		for(int i=1; i<numOfParticles; i++){
			cdf[i] = cdf[i-1] + particles[i].weight;
		}
		
		double u = rand.nextDouble() / numOfParticles;
		double step = 1.0/numOfParticles;
		int i=0;
		for(int j=0; j<numOfParticles; j++){
			while(u>cdf[i]) i++;
			resampleBuffer[j] = new Particle(particles[i].vehicle.copy(), 1.0/numOfParticles);
//			System.out.printf("No.%d is sampled with weight %f\n", i, particles[i].weight);
			u += step;
		}
//		System.out.println("==========================");
		Particle[] swap = particles;
		particles = resampleBuffer;
		resampleBuffer = swap;
		
//		this.pirntPrior();
//		this.pirntPosterior();
	}
	
	/**
	 * return a vehicle with weighted average position/orientation/speed
	 * @return
	 */
	private void interence(){
		Point2D center = new Point2D(0, 0);
		Vector direction = new Vector(0, 0);
		double speed=0;
		for(int i=0; i<numOfParticles; i++){
			center.x += particles[i].vehicle.center.x * particles[i].weight;
			center.y +=  particles[i].vehicle.center.y * particles[i].weight;
			direction.x +=  particles[i].vehicle.direction.x * particles[i].weight;
			direction.y +=  particles[i].vehicle.direction.y * particles[i].weight;
			speed +=  particles[i].vehicle.speed * particles[i].weight;
		}
		this.avgVehicle = new VehicleModel(center, direction, VehicleModel.default_width, VehicleModel.default_length, speed);
		//this.printPosterior();
	}
	
	public boolean isTerminate(){
		return numOfLost>3;
	}
	
	public boolean isLost(){
		return lost;
	}
	
	public Point2D[] getCenters(){
		return centers;
	}
	
	public double getWeight(int idx){
		return particles[idx].weight;
	}
	
	public VehicleModel getVehicleModel(int idx){
		return particles[idx].vehicle;
	}
	
	public VehicleModel getOriginalVehicle(){
		return origin;
	}
	
	public VehicleModel getAvgVehicle(){
		return avgVehicle;
	}
	
	public Point2D getCenter(){
		return avgVehicle.center;
	}
	
	public double getVariance(){
		double Neff=0;
		for(int i=0; i<numOfParticles; i++){
			Neff+=(particles[i].weight*particles[i].weight);
		}
		return Neff;
	}
	
	public VehicleModel[] getPriorVehicle(double ratio){
		int topk = (int)(numOfParticles*ratio);
		ArrayList<VehicleModel> vehicles = new ArrayList<VehicleModel>(topk);
		for(int i=numOfParticles-1; i>numOfParticles-topk; i--){
//			if(this.resampleBuffer[i].weight==0) continue;
			vehicles.add(this.resampleBuffer[i].vehicle);
		}
		return vehicles.toArray(new VehicleModel[vehicles.size()]);
	}
	
	public VehicleModel[] getPosteriorVehicle(){
		VehicleModel[] vehicles = new VehicleModel[numOfParticles];
		for(int i=0; i<numOfParticles; i++){
			vehicles[i] = this.particles[i].vehicle;
		}
		return vehicles;
	}
	
	public Point2D[] getPrior(double ratio){
		int topk = (int)(numOfParticles*ratio);
		ArrayList<Point2D> points = new ArrayList<Point2D>(topk);
		for(int i=numOfParticles-1; i>numOfParticles-topk; i--){
			points.add(this.resampleBuffer[i].vehicle.center);
		}
		return points.toArray(new Point2D[points.size()]);
	}
	
	public Point2D[] getPosterior(){
		Point2D[] points = new Point2D[numOfParticles];
		for(int i=0; i<numOfParticles; i++){
			points[i] = this.particles[i].vehicle.center;
		}
		return points;
	}
	
	public void printPrior(){
		System.out.println();
		for(int i=0; i<numOfParticles; i++){
			if(i%10==0) System.out.println();
			System.out.printf("%f,",this.resampleBuffer[i].weight);
		}
	}
	
	public void printPosterior(){
		System.out.println();
		for(int i=0; i<numOfParticles; i++){
			if(i%10==0) System.out.println();
			System.out.printf("%f,",this.particles[i].weight);
		}
	}
	
}
class Particle{
	VehicleModel vehicle;
	double weight;
	public Particle(VehicleModel v, double w) {
		vehicle = v;
		weight = w;
	}
}

class ParticleComp implements Comparator<Particle>{
	@Override
	public int compare(Particle o1, Particle o2) {
		// TODO Auto-generated method stub
		return (o1.weight>o2.weight ? 1 : (o1.weight==o2.weight ? 0 : -1));
	}
}
