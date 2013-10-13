package prepocess;

import VelodyneDataIO.VirtualTable;

public class BasicFilter {
	protected boolean[][] mask;//mask with same dimension as virtual table
	protected int rows;
	protected int cols;
	
	public BasicFilter(VirtualTable vt) {
		rows=vt.getRowNum();
		cols=vt.getColNum();
		this.mask=new boolean[rows][cols];
	}
	
	public boolean[][] getMask(){
		return mask;
	}
}
