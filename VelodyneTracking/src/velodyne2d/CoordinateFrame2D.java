package velodyne2d;

import java.io.DataInputStream;
import java.io.IOException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

import VelodyneDataIO.Point3D;

import calibration.CoordinateFrame;

public class CoordinateFrame2D {
	protected SimpleMatrix w_R = SimpleMatrix.wrap(new DenseMatrix64F(2, 2));//rotation
	protected SimpleMatrix w_T = SimpleMatrix.wrap(new DenseMatrix64F(2, 1));//translation
	protected SimpleMatrix w_Trans = SimpleMatrix.wrap(new DenseMatrix64F(3, 3));//transformation
	
	//protected double yaw;
	
	public static CoordinateFrame2D readFromStream(DataInputStream dis) throws IOException{
		double[] params = new double[6];
		for (int i=0; i<params.length; i++){
			params[i]=dis.readFloat();
		}
		return new CoordinateFrame2D(params);
	}
	
	public static CoordinateFrame2D fromCoordinateFrame3D(CoordinateFrame frame, boolean reverse){
		return reverse ? 
				new CoordinateFrame2D(new double[] {frame.getPosition().x,frame.getPosition().y, 
													frame.getRotation().get(0, 1), frame.getRotation().get(0, 0),
													frame.getRotation().get(1, 1), frame.getRotation().get(1, 0)} ) :
				new CoordinateFrame2D(new double[] {frame.getPosition().x,frame.getPosition().y, 
													frame.getRotation().get(0, 0), frame.getRotation().get(0, 1),
													frame.getRotation().get(1, 0), frame.getRotation().get(1, 1)} );
	}
	
	public CoordinateFrame2D(){}
	
	/**
	 * 
	 * @param params x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33
	 */
	public CoordinateFrame2D(double params[]){
		w_T.set(0, params[0]);w_T.set(1, params[1]);
		w_Trans.set(0, 2, params[0]);w_Trans.set(1, 2, params[1]);
		if(params.length==6){
			//row first
			w_R.set(0, 0, params[2]);w_R.set(0, 1, params[3]);
			w_R.set(1, 0, params[4]);w_R.set(1, 1, params[5]);
			
			w_Trans.set(0, 0, params[2]);w_Trans.set(0, 1, params[3]);
			w_Trans.set(1, 0, params[4]);w_Trans.set(1, 1, params[5]);
			w_Trans.set(2,0,0);w_Trans.set(2,1,0);w_Trans.set(2,2,1);
		}
	}
	
	public Point2D getPosition(){
		return new Point2D(w_T.get(0), w_T.get(1));
	}
	
	/*
	 * return yaw in radius unit, from world(NED) to body(NED), clock-wise
	 */
	public double getYaw(){
		return Math.atan2(w_R.get(1,0), w_R.get(0,0));
	}
	
	public SimpleMatrix getRotation(){
		return w_R;
	}
	
	public SimpleMatrix getTranslation(){
		return w_T;
	}
	
	public void copyRotation(SimpleMatrix rot){
		w_R.set(rot);
		this.w_Trans.insertIntoThis(0, 0, w_R);
	}
	
	public void copyTranslation(SimpleMatrix T){
		w_T.set(T);
		this.w_Trans.insertIntoThis(0, 2, w_T);
		this.w_Trans.set(2,2,1);
	}
	
	public CoordinateFrame cloneFrame(){
		return new CoordinateFrame(new double[] {w_T.get(0), w_T.get(1), w_R.get(0, 0), w_R.get(0, 1), w_R.get(1, 0), w_R.get(1, 1)});
	}
	
	public SimpleMatrix getTransMatrix(){
		return w_Trans;
	}
	
	public Vector getX(){
		return new Vector(w_R.get(0, 0), w_R.get(1, 0));
	}
	
	public Vector getY(){
		return new Vector(w_R.get(0, 1), w_R.get(1, 1));
	}
}
