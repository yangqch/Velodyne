package calibration;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.lang.Math;

import org.ejml.simple.SimpleMatrix;

import velodyne2d.Point2D;
import velodyne2d.Vector;

public class BodyFrame extends CoordinateFrame{
	double roll, pitch, yaw;//in radius
	
	public static BodyFrame readFromStream(DataInputStream dis, boolean attitudeOrMatrix) throws IOException{
		int num = attitudeOrMatrix ? 6 : 12;
		double[] params = new double[num];
		for (int i=0; i<num; i++){
			params[i]=dis.readFloat();
		}
		return new BodyFrame(params);
	}
	
	public BodyFrame(double[] params) {
		w_T.set(0, params[0]);w_T.set(1, params[1]);w_T.set(2, params[2]);
		w_Trans.set(0, 3, params[0]);w_Trans.set(1, 3, params[1]);w_Trans.set(2, 3, params[2]);
		if(params.length==6){
			this.makeRotationMatrix(params[3], params[4], params[5]);
		}else if(params.length==12){
			//row first
			w_R.set(0, 0, params[3]);w_R.set(0, 1, params[4]);w_R.set(0, 2, params[5]);
			w_R.set(1, 0, params[6]);w_R.set(1, 1, params[7]);w_R.set(1, 2, params[8]);
			w_R.set(2, 0, params[9]);w_R.set(2, 1, params[10]);w_R.set(2, 2, params[11]);
			
			w_Trans.set(0, 0, params[3]);w_Trans.set(0, 1, params[4]);w_Trans.set(0, 2, params[5]);
			w_Trans.set(1, 0, params[6]);w_Trans.set(1, 1, params[7]);w_Trans.set(1, 2, params[8]);
			w_Trans.set(2, 0, params[9]);w_Trans.set(2, 1, params[10]);w_Trans.set(2, 2, params[11]);
			w_Trans.set(3,0,0);w_Trans.set(3,1,0);w_Trans.set(3,2,0);w_Trans.set(3,3,1);
		}else{
			
		}
	}
	
	public BodyFrame(CoordinateFrame newFrame){
		this.w_T = new SimpleMatrix(newFrame.w_T);
		this.w_R = new SimpleMatrix(newFrame.w_R);
		this.w_Trans = new SimpleMatrix(newFrame.w_Trans);
	}
	
	/**
	 * get yaw from rotation matrix
	 * @param isNED, true-NED, false-NWU
	 * @return
	 */
	public double getYaw(boolean isNED){
		return Math.atan2(this.w_R.get(0, 0), this.w_R.get(1, 0));
	}
	
	public Point2D getPosion2D(){
		return new Point2D(w_T.get(0), w_T.get(1));
	}
	
	
	/**
	 * fix!
	 * @return
	 */
	public Vector getBodyX2D(){
		return new Vector(this.w_R.get(0, 0), this.w_R.get(1, 0));
	}
	
	public Vector getBodyY2D(){
		return new Vector(this.w_R.get(0, 1), this.w_R.get(1, 1));
	}
	
	private void makeRotationMatrix(double roll, double pitch, double yaw){
		this.roll = roll * Math.PI /180;
		this.pitch = pitch * Math.PI /180;
		this.yaw = yaw * Math.PI /180;
		
		w_R.set(0, 0, Math.cos(this.pitch)*Math.cos(this.yaw));
		w_R.set(0, 1,-Math.cos(this.roll)*Math.sin(this.yaw) + Math.sin(this.roll)*Math.sin(this.pitch)*Math.cos(this.yaw));
		w_R.set(0, 2, Math.sin(this.roll)*Math.sin(this.yaw) + Math.cos(this.roll)*Math.sin(this.pitch)*Math.cos(this.yaw));
		w_R.set(1, 0, Math.cos(this.pitch)*Math.sin(this.yaw));
		w_R.set(1, 1, Math.cos(this.roll)*Math.cos(this.yaw) + Math.sin(this.roll)*Math.sin(this.pitch)*Math.sin(this.yaw));
		w_R.set(1, 2,-Math.sin(this.roll)*Math.cos(this.yaw) + Math.cos(this.roll)*Math.sin(this.pitch)*Math.sin(this.yaw));
		w_R.set(2, 0,-Math.sin(this.pitch));
		w_R.set(2, 1, Math.sin(this.roll)*Math.cos(this.pitch));
		w_R.set(2, 2, Math.cos(this.roll)*Math.cos(this.pitch));
		
		//w_R = w_R.transpose();
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				w_Trans.set(i,j,w_R.get(i,j));
			}
		}
		w_Trans.set(3,0,0);w_Trans.set(3,1,0);w_Trans.set(3,2,0);w_Trans.set(3,3,1);
	}
	
	public void printAttitudeBinaryToFile(DataOutputStream dos) throws IOException {
		dos.writeFloat((float)w_T.get(0));
		dos.writeFloat((float)w_T.get(1));
		dos.writeFloat((float)w_T.get(2));
		
		dos.writeFloat((float)this.roll);
		dos.writeFloat((float)this.pitch);
		dos.writeFloat((float)this.yaw);
		
		for(int i=0;i<6;i++)
			dos.writeFloat(0.f);
	}
	
	@Override
	public String toString(){
		StringBuilder sb = new StringBuilder();
		sb.append("N:"); sb.append((float)w_T.get(0)); sb.append('\t');
		sb.append("ROLL:"); sb.append((float)this.roll); sb.append('\n');
		
		sb.append("E:"); sb.append((float)w_T.get(1)); sb.append('\t');
		sb.append("PITCH:"); sb.append((float)this.pitch); sb.append('\n');
		
		sb.append("D:"); sb.append((float)w_T.get(2)); sb.append('\t');
		sb.append("YAW:"); sb.append((float)this.yaw); sb.append('\n');
		
		return sb.toString();
	}
}
