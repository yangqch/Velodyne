package calibration;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;

import VelodyneDataIO.Point3D;

public class FrameTransformer {
	
	static final SimpleMatrix worldframe = SimpleMatrix.identity(4);
	
	public FrameTransformer() {
	}
	
	public SimpleMatrix make3DDataMatrix(Point3D[] point, int num){
		SimpleMatrix dataset = SimpleMatrix.wrap(new DenseMatrix64F(3, num));
		for(int i=0; i<num; i++){
			dataset.set(0, i, point[i].x);dataset.set(1, i, point[i].y);dataset.set(2, i, point[i].z);
		}
		return dataset;
	}
	
	public SimpleMatrix make3DDataMatrix(double[] data, int dim, int num){
		SimpleMatrix dataset = SimpleMatrix.wrap(new DenseMatrix64F(dim, num));
		for(int i=0; i<num; i++){
			dataset.set(0, i, data[i*dim]);dataset.set(1, i, data[i*dim+1]);dataset.set(2, i, data[i*dim+2]);
		}
		return dataset;
	}
	
	public SimpleMatrix make4DDataMatrix(float[] data){
		int num = data.length/3;
		SimpleMatrix dataset = SimpleMatrix.wrap(new DenseMatrix64F(4, num));
		for(int i=0; i<num; i++){
			dataset.set(0, i, data[i*3]);dataset.set(1, i, data[i*3+1]);dataset.set(2, i, data[i*3+2]);
			dataset.set(3, i, 1);
		}
		return dataset;
	}
	
	public SimpleMatrix make4DDataMatrix(Point3D[] points){
		int num = points.length;
		SimpleMatrix dataset = SimpleMatrix.wrap(new DenseMatrix64F(4, num));
		for(int i=0; i<num; i++){
			dataset.set(0, i, points[i].x);dataset.set(1, i, points[i].y);dataset.set(2, i, points[i].z);
			dataset.set(3, i, 1);
		}
		return dataset;
	}
	
	public Point3D[] transform(CoordinateFrame oldFrame, CoordinateFrame newFrame, SimpleMatrix dataset){
		
		//make rotation and translation 
		SimpleMatrix new_T_old = newFrame.getRotation().transpose().mult(oldFrame.getTranslation().minus(newFrame.getTranslation()));
		SimpleMatrix new_R_old = newFrame.getRotation().transpose().mult(oldFrame.getRotation());
		
		SimpleMatrix new_point = new_R_old.mult(dataset);
		
		int num = dataset.numCols();
		SimpleMatrix x = SimpleMatrix.wrap(new DenseMatrix64F(1, num));
		SimpleMatrix y = SimpleMatrix.wrap(new DenseMatrix64F(1, num));
		SimpleMatrix z = SimpleMatrix.wrap(new DenseMatrix64F(1, num));
		
		CommonOps.add(new_point.extractVector(true, 0).getMatrix(), -new_T_old.get(0), x.getMatrix());
		CommonOps.add(new_point.extractVector(true, 1).getMatrix(), -new_T_old.get(0), y.getMatrix());
		CommonOps.add(new_point.extractVector(true, 2).getMatrix(), -new_T_old.get(0), z.getMatrix());
		
		Point3D[] new_p = new Point3D[num];
		for(int i=0; i<num; i++){
			new_p[i]=new Point3D(x.get(i), y.get(i), z.get(i));
		}

		return new_p;
	}
	
	public Point3D[] transform4D(CoordinateFrame oldFrame, CoordinateFrame newFrame, SimpleMatrix old_dataset){
		SimpleMatrix w_Trans_old = oldFrame==null ? worldframe : oldFrame.getTransMatrix();
		SimpleMatrix new_Trans_w = newFrame==null ? worldframe : newFrame.getTransMatrix().invert();
		
		SimpleMatrix new_dataset = new_Trans_w.mult(w_Trans_old).mult(old_dataset);
		
		int num = new_dataset.numCols();
		Point3D[] new_p = new Point3D[num];
		for(int i=0; i<num; i++){
			new_p[i]=new Point3D(new_dataset.get(0, i), new_dataset.get(1, i), new_dataset.get(2, i));
		}	
		return new_p;
	}
	
	public Point3D transform4D(CoordinateFrame oldFrame, CoordinateFrame newFrame, Point3D old_point){
		SimpleMatrix w_Trans_old = oldFrame==null ? worldframe : oldFrame.getTransMatrix();
		SimpleMatrix new_Trans_w = newFrame==null ? worldframe : newFrame.getTransMatrix().invert();
		
		SimpleMatrix old_data = SimpleMatrix.wrap(new DenseMatrix64F(4, 1));
		old_data.set(0, old_point.x); old_data.set(1, old_point.y); old_data.set(2, old_point.z);
		old_data.set(3, 1);
		
		SimpleMatrix new_data = new_Trans_w.mult(w_Trans_old).mult(old_data);
			
		return new Point3D(new_data.get(0), new_data.get(1), new_data.get(2));
	}
	/**
	 * transform old_dataset from its current frame to a new frame using transFrame(new_Trans_old)
	 * @param transFrame, coordinate frame(with T, R, Trans matrix) from old frame to new frame
	 * @param old_dataset
	 * @return
	 */
	public Point3D[] transform4D(CoordinateFrame transFrame, SimpleMatrix old_dataset){
		SimpleMatrix new_Trans_L = transFrame.getTransMatrix();
		
		SimpleMatrix new_dataset = new_Trans_L.mult(old_dataset);
		
		int num = new_dataset.numCols();
		Point3D[] new_p = new Point3D[num];
		for(int i=0; i<num; i++){
			new_p[i]=new Point3D(new_dataset.get(0, i), new_dataset.get(1, i), new_dataset.get(2, i));
		}	
		return new_p;
	}
	
	public Point3D[] transform4D(CoordinateFrame oldFrame, CoordinateFrame newFrame, float[] data){
		SimpleMatrix old_dataset = this.make4DDataMatrix(data);
		return this.transform4D(oldFrame, newFrame, old_dataset);
	}
	
	public Point3D[] transform4D(CoordinateFrame transFrame, float[] data){
		SimpleMatrix old_dataset = this.make4DDataMatrix(data);
		return this.transform4D(transFrame, old_dataset);
	}
	
	public Point3D[] transform4D(CoordinateFrame oldFrame, CoordinateFrame newFrame, Point3D[] data){
		SimpleMatrix old_dataset = this.make4DDataMatrix(data);
		return this.transform4D(oldFrame, newFrame, old_dataset);
	}
	
//	public Point3D[] transform4DwithNull(CoordinateFrame oldFrame, CoordinateFrame newFrame, Point3D[] data){
//		ArrayList<Point3D> validData = new ArrayList<Point3D>();
//		for(Point3D p: data){
//			if(p!=null) validData.add(p);
//		}
//		SimpleMatrix old_dataset = this.make4DDataMatrix(validData.toArray(new Point3D[validData.size()]));
//		Point3D[] transformedData = this.transform4D(oldFrame, newFrame, old_dataset);
//		for(Point3D p: data){
//			if(p!=null) validData.add(p);
//		}
//	}
	
	/**
	 * transform bodyFrame to newFrame using tranFrame
	 * @param bodyFrame
	 * @param transFrame: the transformation from L to new
	 */
	public void transformBodyFrame(BodyFrame bodyFrame, CoordinateFrame transFrame){
		SimpleMatrix L_T_new = transFrame.getTranslation();//vector L->new w.r.t L
		SimpleMatrix new_R_L = transFrame.getRotation();
		SimpleMatrix w_T_L = bodyFrame.getTranslation();//vector w->L w.r.t W
		SimpleMatrix w_R_L = bodyFrame.getRotation();
		
		bodyFrame.copyTranslation(w_T_L.plus(w_R_L.mult(L_T_new)));
		bodyFrame.copyRotation(w_R_L.mult(new_R_L.invert()));
	}
	
	public Point3D calcMean(Point3D[] points){
		int num = points.length;
		Point3D avg=new Point3D(0, 0, 0);
		for(int i=0;i<points.length;i++){
			avg.x+=points[i].x/num;
			avg.y+=points[i].y/num;
			avg.z+=points[i].z/num;
		}
		return avg;
	}
	
	public CoordinateFrame getWorldFrame(){
		return new CoordinateFrame(new double[] {0,0,0, 1,0,0, 0,1,0, 0,0,1});
	}
	
	static public void main(String[] args){
		FrameTransformer trans  = new FrameTransformer();
		
	}
	
}
