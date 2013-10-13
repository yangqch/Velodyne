package detection;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.SingularOps;
import org.ejml.simple.SimpleMatrix;

import velodyne2d.Line;
import velodyne2d.Point2D;
import velodyne2d.Vector;

import VelodyneDataIO.Point3D;

public class PCALineFitter {

	private int dataDimension=2;
	private int sampleNum;
	private DenseMatrix64F A = new DenseMatrix64F(1, 1);//data
	private DenseMatrix64F prinComps = new DenseMatrix64F(dataDimension, dataDimension);// = new DenseMatrix64F(dataDimension, dataDimension);//principle components
	private DenseMatrix64F singValue = new DenseMatrix64F(dataDimension, dataDimension);;
	private DenseMatrix64F mean = new DenseMatrix64F(2, 1);//n(dim)-by-1
	private SimpleMatrix o;//wrapper of mean
	private SimpleMatrix comps;//wrapper of prinComps, n(dim)-by-n(dim)
	private SimpleMatrix comp1, comp2;//1st, 2nd components, //n(dim)-by-1
	private double variance;//1nd singular value, variance of data on the 2nd principle component
	
	public Line fitOneLinePCA(Point2D[] points, int start, int end){
		A.reshape(2, end-start);
		this.PCA(points, start, end);
		Point2D new_p1 = this.project(o, comp1, points[start]);
		Point2D new_p2 = this.project(o, comp1, points[end-1]);
		return new Line(new_p1, new_p2);
	}
	
	public Line[] fitTwoLinePCA(Point2D[] points, int breakIdx){
		Line[] lines = new Line[2];
		//PCA on first part
		A.reshape(2, breakIdx+1);
		this.PCA(points, 0, breakIdx+1);
		double variace1 = this.variance;
		SimpleMatrix o1 = this.o.copy();
		SimpleMatrix v1 = this.comp1.copy();
		SimpleMatrix u1 = this.comp2.copy();
		
		//PCA on second part
		A.reshape(2, points.length-breakIdx);
		this.PCA(points, breakIdx, points.length);
		double variace2 = this.variance;
		SimpleMatrix o2 = this.o.copy();
		SimpleMatrix v2 = this.comp1.copy();
		SimpleMatrix u2 = this.comp2.copy();
		
		//new point2 should be the intersection of these two lines
		if(variace1 <= variace2){//trust PCA from point group 1
			Point2D new_p1 = this.project(o1, v1, points[0]);
			Point2D new_p3 = this.project(o2, u1, points[points.length-1]);
			Point2D new_p2 = this.findIntersection(o1, v1, o2, u1);
//			
//			System.out.println(o1);
//			System.out.println(v1);
//			System.out.println(o2);
//			System.out.println(u2);
//			System.out.println(points[0]);
//			System.out.println(new_p1);
//			System.out.println(points[points.length-1]);
//			System.out.println(new_p3);
//			System.out.println(new_p2);
			
			lines[0] = new Line(new_p2, new_p1);
			lines[1] = new Line(new_p2, new_p3);
		}else{//trust PCA from point group 2
			Point2D new_p1 = this.project(o1, u2, points[0]);
			Point2D new_p3 = this.project(o2, v2, points[points.length-1]);
			Point2D new_p2 = this.findIntersection(o1, u2, o2, v2);
			lines[0] = new Line(new_p2, new_p1);
			lines[1] = new Line(new_p2, new_p3);
		}
		return lines;
	}
	
	/**
	 * find intersection of two lines
	 * o1 along v1, o2 along v2
	 * @return
	 */
	public Point2D findIntersection(SimpleMatrix o1, SimpleMatrix v1, SimpleMatrix o2, SimpleMatrix v2){
		SimpleMatrix tmp = new SimpleMatrix(new double[][] {{-v1.get(0), v2.get(0)}, {-v1.get(1), v2.get(1)}});//2-by-2
		SimpleMatrix res = tmp.invert().mult(o1.minus(o2));//2-by-1
		SimpleMatrix p = o1.plus(v1.scale(res.get(0)));
		
//		System.out.println(v1);System.out.println(v2);System.out.println(tmp);
//		System.out.println(tmp.invert());System.out.println(o1.minus(o2));System.out.println(res);
//		System.out.println(o1.plus(v1.scale(res.get(0))));
		
		return new Point2D(p.get(0), p.get(1));
	}
	
	/**
	 * project p to vector v with o as orgin
	 * @param o
	 * @param v
	 * @param p
	 * @return projected point
	 */
	private Point2D project(SimpleMatrix o, SimpleMatrix v, Point2D point){
		SimpleMatrix p = new SimpleMatrix(new double[][] {{point.x}, {point.y}});
		SimpleMatrix proj = v.scale(p.minus(o).dot(v)).plus(o);
		return new Point2D(proj.get(0), proj.get(1));
	}
	/**
	 * PCA on points, [idx1, idx2)
	 * @param points
	 * @param idx1
	 * @param idx2
	 */
	private void PCA(Point2D[] points, int idx1, int idx2){
		this.sampleNum = idx2 - idx1;
		//set points value to A
		for(int i=0; i<this.sampleNum; i++){
			A.set(0, i, points[idx1+i].x);
			A.set(1, i, points[idx1+i].y);
		}
		//compute mean of each dimension
		double[] sum = new double[this.dataDimension];
		for(int i=0; i<this.dataDimension; i++){
			for(int j=0; j<this.sampleNum; j++){
				sum[i]+=A.get(i, j);
			}
		}
		for(int i=0; i<this.dataDimension; i++){
			mean.set(i, 0, sum[i]/this.sampleNum);
		}
		//subtract mean from A 
		for(int i=0; i<this.dataDimension; i++){
			for(int j=0; j<this.sampleNum; j++){
				A.set(i, j, A.get(i, j) - mean.get(i, 0));
			}
		}
		//create svd 
		SingularValueDecomposition<DenseMatrix64F> svd = 
				DecompositionFactory.svd(this.dataDimension, this.sampleNum, true, false, true);
		if(! svd.decompose(A))
			throw new RuntimeException("SVD failed");
		
		svd.getU(this.prinComps, false);
		svd.getW(this.singValue);
		
		SingularOps.descendingOrder(this.prinComps, false, this.singValue, null, false);
		
		this.comps = new SimpleMatrix(prinComps);
		this.comp1 = comps.extractVector(false, 0);//1st column
		this.comp2 = comps.extractVector(false, 1);//2nd column
		this.variance = this.singValue.get(1, 1); //singular value of 2nd column, variance of data on the less principle component
		this.o = new SimpleMatrix(mean);
	}
}
