package gridmap_generic;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.RandomAccessFile;
import java.io.Serializable;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;

public class Gridmap<T extends Cell> implements Serializable{
	Index2D index;//index in GridmapMatrix
	double org_x, org_y;//left-upper corner(north, west)
	GridmapAttribute attr;
	
	Cell[] cells;
	
	static <T extends Cell> Gridmap<T> loadFromFile(File f, GridmapAttribute attr) throws IOException, ClassNotFoundException{
		Gridmap<T> map = new Gridmap<T>(attr);
		ObjectInputStream stream = new ObjectInputStream(new FileInputStream(f));
		map.index=(Index2D)stream.readObject();
		map.org_x=stream.readDouble();
		map.org_y=stream.readDouble();
		for(int i=0; i<attr.numOfCells; i++){
			map.cells[i]=(T)stream.readObject();
		}
		stream.close();
		return map;
	}
	
	void dumpToFile(File f) throws IOException, ClassNotFoundException{
		ObjectOutputStream stream = new ObjectOutputStream(new FileOutputStream(f));
		stream.writeObject(index);
		stream.writeDouble(org_x);
		stream.writeDouble(org_y);
		for(int i=0; i<attr.numOfCells; i++){
			stream.writeObject(cells[i]);
		}
		stream.close();
	}
	/**
	 * Constructor called when Gridmap is first used
	 * @param org_x
	 * @param org_y
	 * @param attr
	 * @param sample
	 */
	public Gridmap(double org_x, double org_y, GridmapAttribute attr, T sample) {
		this.org_x = org_x;
		this.org_y = org_y;
		this.attr = attr;
		this.cells = new Cell[attr.numOfCells];
		for(int i=0; i<attr.numOfCells; i++){
			this.cells[i]=sample.cloneCell();
		}
	}
	/**
	 * Constructor called when Gridmap needed to be loaded from file
	 * only called by static method loadFromFile
	 * @param attr
	 */
	private Gridmap(GridmapAttribute attr) {
		this.attr = attr;
		this.cells = new Cell[attr.numOfCells];
	}
	
	public double[][] getCellValues(){
		double[][] data = new double[attr.rows][attr.cols];
		for(int i=0; i<attr.rows; i++){
			for(int j=0; j<attr.cols; j++){
				double v = cells[i*attr.cols+j].getValue();
				data[i][j] = v;
			}
		}
		return data;
	}
	
	@Override
	public String toString() {
		return attr.toString();
	}
	
	public T getCell(int row, int col){
		return (T)cells[row*attr.cols + col];
	}
}
