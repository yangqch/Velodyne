package calibration;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

import VelodyneDataIO.Point3D;

public class CoordinateFrame {
	protected SimpleMatrix w_R = SimpleMatrix.wrap(new DenseMatrix64F(3, 3));//rotation
	protected SimpleMatrix w_T = SimpleMatrix.wrap(new DenseMatrix64F(3, 1));//translation
	protected SimpleMatrix w_Trans = SimpleMatrix.wrap(new DenseMatrix64F(4, 4));//transformation
	
	public static CoordinateFrame readFromStream(DataInputStream dis) throws IOException{
		double[] params = new double[12];
		for (int i=0; i<params.length; i++){
			params[i]=dis.readFloat();
		}
		return new CoordinateFrame(params);
	}
	
	public CoordinateFrame(){}
	
	/**
	 * 
	 * @param params x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33
	 */
	public CoordinateFrame(double params[]){
		w_T.set(0, params[0]);w_T.set(1, params[1]);w_T.set(2, params[2]);
		w_Trans.set(0, 3, params[0]);w_Trans.set(1, 3, params[1]);w_Trans.set(2, 3, params[2]);
		if(params.length==12){
			//row first
			w_R.set(0, 0, params[3]);w_R.set(0, 1, params[4]);w_R.set(0, 2, params[5]);
			w_R.set(1, 0, params[6]);w_R.set(1, 1, params[7]);w_R.set(1, 2, params[8]);
			w_R.set(2, 0, params[9]);w_R.set(2, 1, params[10]);w_R.set(2, 2, params[11]);
			
			w_Trans.set(0, 0, params[3]);w_Trans.set(0, 1, params[4]);w_Trans.set(0, 2, params[5]);
			w_Trans.set(1, 0, params[6]);w_Trans.set(1, 1, params[7]);w_Trans.set(1, 2, params[8]);
			w_Trans.set(2, 0, params[9]);w_Trans.set(2, 1, params[10]);w_Trans.set(2, 2, params[11]);
			w_Trans.set(3,0,0);w_Trans.set(3,1,0);w_Trans.set(3,2,0);w_Trans.set(3,3,1);
		}
	}
	
	public Point3D getPosition(){
		return new Point3D(w_T.get(0), w_T.get(1), w_T.get(2));
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
		this.w_Trans.insertIntoThis(0, 3, w_T);
		this.w_Trans.set(3,3,1);
	}
	
	public CoordinateFrame cloneFrame(){
		return new CoordinateFrame(new double[] {w_T.get(0), w_T.get(1), w_T.get(2), w_R.get(0, 0), w_R.get(0, 1),w_R.get(0, 2), w_R.get(1, 0), w_R.get(1, 1),w_R.get(1, 2), w_R.get(2, 0), w_R.get(2, 1),w_R.get(2, 2),});
	}
	
	public SimpleMatrix getTransMatrix(){
		return w_Trans;
	}
	
	public void printBinaryToFile(DataOutputStream dos) throws IOException {
		dos.writeFloat((float)w_T.get(0));
		dos.writeFloat((float)w_T.get(1));
		dos.writeFloat((float)w_T.get(2));
		
		dos.writeFloat((float)w_R.get(0,0));
		dos.writeFloat((float)w_R.get(0,1));
		dos.writeFloat((float)w_R.get(0,2));
		dos.writeFloat((float)w_R.get(1,0));
		dos.writeFloat((float)w_R.get(1,1));
		dos.writeFloat((float)w_R.get(1,2));
		dos.writeFloat((float)w_R.get(2,0));
		dos.writeFloat((float)w_R.get(2,1));
		dos.writeFloat((float)w_R.get(2,2));
	}
	
	public String toString(){
		StringBuilder sb = new StringBuilder();
		sb.append((float)w_T.get(0));sb.append(',');
		sb.append((float)w_T.get(1));sb.append(',');
		sb.append((float)w_T.get(2));sb.append(',');
		sb.append((float)w_R.get(0,0));sb.append(',');
		sb.append((float)w_R.get(0,1));sb.append(',');
		sb.append((float)w_R.get(0,2));sb.append(',');
		sb.append((float)w_R.get(1,0));sb.append(',');
		sb.append((float)w_R.get(1,1));sb.append(',');
		sb.append((float)w_R.get(1,2));sb.append(',');
		sb.append((float)w_R.get(2,0));sb.append(',');
		sb.append((float)w_R.get(2,1));sb.append(',');
		sb.append((float)w_R.get(2,2));sb.append('\n');
		return sb.toString();
	}
}
