package velodyne2d;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

import VelodyneDataIO.Point3D;
import calibration.CoordinateFrame;

public class FrameTransformer2D {
	static final SimpleMatrix worldframe = SimpleMatrix.identity(3);
	
	private SimpleMatrix makeDataMatrix(Point2D[] points){
		int num = points.length;
		SimpleMatrix dataset = SimpleMatrix.wrap(new DenseMatrix64F(3, num));
		for(int i=0; i<num; i++){
			dataset.set(0, i, points[i].x);dataset.set(1, i, points[i].y);
			dataset.set(2, i, 1);
		}
		return dataset;
	}
	
	private SimpleMatrix makeDataMatrix(ArrayList<Point2D> points){
		int num = points.size();
		SimpleMatrix dataset = SimpleMatrix.wrap(new DenseMatrix64F(3, num));
		for(int i=0; i<num; i++){
			dataset.set(0, i, points.get(i).x);dataset.set(1, i, points.get(i).y);
			dataset.set(2, i, 1);
		}
		return dataset;
	}
	
	private Point2D[] transform(CoordinateFrame2D oldFrame, CoordinateFrame2D newFrame, SimpleMatrix dataset){
		SimpleMatrix w_Trans_old = oldFrame==null ? worldframe : oldFrame.getTransMatrix();
		SimpleMatrix new_Trans_w = newFrame==null ? worldframe : newFrame.getTransMatrix().invert();
		
		SimpleMatrix new_dataset = new_Trans_w.mult(w_Trans_old).mult(dataset);
		
		int num = new_dataset.numCols();
		Point2D[] new_p = new Point2D[num];
		for(int i=0; i<num; i++){
			new_p[i]=new Point2D(new_dataset.get(0, i), new_dataset.get(1, i));
		}	
		return new_p;
	}
	
	/**
	 * transform 2D points from oldFrame to newFrame
	 * oldFrame and newFrame will be considered as world frame if null is passed in  
	 * @param oldFrame
	 * @param newFrame
	 * @param points
	 * @return
	 */
	public Point2D[] transform(CoordinateFrame2D oldFrame, CoordinateFrame2D newFrame, Point2D[] points){
		return transform(oldFrame, newFrame, this.makeDataMatrix(points));
	}
	
	public Point2D[] transform(CoordinateFrame2D oldFrame, CoordinateFrame2D newFrame, ArrayList<Point2D> points){
		return transform(oldFrame, newFrame, this.makeDataMatrix(points));
	}
	
	public Point2D transform(CoordinateFrame2D oldFrame, CoordinateFrame2D newFrame, Point2D point){
		SimpleMatrix w_Trans_old = oldFrame==null ? worldframe : oldFrame.getTransMatrix();
		SimpleMatrix new_Trans_w = newFrame==null ? worldframe : newFrame.getTransMatrix().invert();
		
		SimpleMatrix old_data = SimpleMatrix.wrap(new DenseMatrix64F(3, 1));
		old_data.set(0, point.x); old_data.set(1, point.y);
		old_data.set(2, 1);
		
		SimpleMatrix new_data = new_Trans_w.mult(w_Trans_old).mult(old_data);
			
		return new Point2D(new_data.get(0), new_data.get(1));
	}
	
	public Line transform(CoordinateFrame2D oldFrame, CoordinateFrame2D newFrame, Line line){
		Point2D p1 = this.transform(oldFrame, newFrame, line.p1);
		Point2D p2 = this.transform(oldFrame, newFrame, line.p2);
		return new Line(p1, p2);
	}
}
