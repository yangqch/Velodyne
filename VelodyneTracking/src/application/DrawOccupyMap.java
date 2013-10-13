package application;

import gridmap_generic.GridmapMatrix;
import gridmap_generic.OccupyCell;

import java.io.File;

public class DrawOccupyMap {
	static void drawMultiOccupyMap(String occupy_map_dir, int row1, int row2, int col1, int col2, double occupyProb){
		File mapFile = new File(occupy_map_dir);
		File imgFile = new File("/home/qichi/Velodyne/map_detect.png");
		OccupyCell emptyCell = new OccupyCell();
		GridmapMatrix<OccupyCell> matrix=null;
		try{
			matrix = GridmapMatrix.loadGridmapMatrix(mapFile, emptyCell, true);
			matrix.drawMultiGridmapImage(row1, row2, col1, col2, imgFile, occupyProb);
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			matrix.finish();
		}
	}

	static void drawOccupyMiniMap(String occupy_map_dir, int i, int j, double occupyProb){
		File mapFile = new File(occupy_map_dir);
		File imgFile = new File(String.format("/home/qichi/Velodyne/%d_%d.png", i, j));
		OccupyCell emptyCell = new OccupyCell();
		GridmapMatrix<OccupyCell> matrix=null;
		try{
			matrix = GridmapMatrix.loadGridmapMatrix(mapFile, emptyCell, true);
			matrix.drawMinGridmapImage(i, j, imgFile, occupyProb);
		}catch(Exception e){
			e.printStackTrace();
		}finally{
			matrix.finish();
		}
	}
	
	public static void main(String[] argv){
		String occupy_map_dir = "";
		int row=-1; 
		int col=-1;
		double occupyProb=0.5;
		try{
			occupy_map_dir = argv[0];
			if(argv.length==4){
				row = Integer.parseInt(argv[1]);
				col = Integer.parseInt(argv[2]);
				occupyProb = Double.parseDouble(argv[3]);
				drawOccupyMiniMap(occupy_map_dir, row, col, occupyProb);
			}else if(argv.length==6){
				int row1 = Integer.parseInt(argv[1]);
				int row2 = Integer.parseInt(argv[2]);
				int col1 = Integer.parseInt(argv[3]);
				int col2 = Integer.parseInt(argv[4]);
				occupyProb = Double.parseDouble(argv[5]);
				drawMultiOccupyMap(occupy_map_dir, row1, row2, col1, col2, occupyProb);
			}else{
				System.out.println("USAGE: java -jar *.jar occupy_filename row col occupyProb(occupy ratio in cell larger than this will be drawn)");
			}
		}catch(Exception e){
			e.printStackTrace();
			System.out.println("USAGE: java -jar *.jar occupy_filename row col occupyProb(occupy ratio in cell larger than this will be drawn)");
			System.exit(-1);
		}
	}
}
