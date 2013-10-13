package gridmap_generic;

public class GridmapAttribute {
	double size_x, size_y;//size in meters
	int rows, cols;//size in cells
	double res_x, res_y;//size of cell in meters
	int numOfCells;
	
	public GridmapAttribute(double size_x, double size_y, int rows, int cols) {
		this.size_x = size_x;
		this.size_y = size_y;
		this.rows = rows;
		this.cols = cols;
		this.res_x = size_x/cols;
		this.res_y = size_y/rows;
		
		numOfCells = rows*cols;
	}
	
	public GridmapAttribute(double size_x, double size_y, double res_x, double res_y) {
		this.size_x = size_x;
		this.size_y = size_y;		
		this.rows = (int)Math.ceil(size_y/res_y);
		this.cols = (int)Math.ceil(size_x/res_x);
		this.res_x = size_x/cols;
		this.res_y = size_y/rows;
		
		numOfCells = rows*cols;
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(String.format("size_x:%.3f\t", size_x));
		sb.append(String.format("size_y:%.3f\n", size_y));
		sb.append(String.format("rows:%d\t", rows));
		sb.append(String.format("cols:%d\n", cols));
		sb.append(String.format("numOfCells:%d\n", numOfCells));
		return sb.toString();
	}
}
