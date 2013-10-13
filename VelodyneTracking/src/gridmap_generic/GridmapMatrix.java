package gridmap_generic;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileInputStream;
import java.io.Serializable;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import javax.imageio.ImageIO;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Attr;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import VelodyneDataIO.LidarFrame;
import VelodyneDataIO.Point3D;

public class GridmapMatrix <T extends Cell>{
	final static int DEFAULT_CACHE_SIZE=7;//cannot be smaller, every time 2 matrices will be updated
	final static float LOAD_FACTOR=.75f;
	final static String META_FILE="meta.xml";
	final static double OCCUPY_PROB = 0.75;
	
	//x is north, y is east
	double org_x, org_y;//origin left-upper(north-west) corner, in meters 
	double size_x, size_y;//size of matrix in meters 
	int rows, cols;//size of matrix in Gridmaps
	double res_x, res_y;
	GridmapAttribute gridmapAttr;//attribute of the gridmap
	private T emptyCell;//sample empty cell for T, used when initializing Gridmap<T>
	
	double occupy_prob = OCCUPY_PROB;
	
	File gridFileDir;//the directory saving GridmapMatrix meta data and Gridmap dump files
	
	GridCache<T> gridCache;//a cache of Gridmaps in data warehouse
	int cacheSize = DEFAULT_CACHE_SIZE;//size of the buffer, when size is full, adding new Gridmap will push oldest Gridmap to be dumped
	boolean readOnly;
	
	Map<Index2D, File> gridIndex;//a hash map of Gridmap index to Gridmap dump file
	
	/**
	 * load gridmap matrix from a directory
	 * @param gridFileDir: directory stored the whole map, containing metadata.xml and all mini gridmaps
	 * @param emptyCell: an example of empty cell that is used when a new map is created 
	 * @param readOnly: if readyOnly=true, when mini grid map is not available, a new one will be created and metadata.xml will be updated
	 * @return a GridmapMatrix will type T (class implements Cell)
	 * @throws Exception
	 */
	public static <T extends Cell> GridmapMatrix<T> loadGridmapMatrix(File gridFileDir, T emptyCell, boolean readOnly) throws Exception{
		if(!gridFileDir.isDirectory())
			throw new Exception(String.format("%s is not a dictionary", gridFileDir));
		File gridMetadata = new File(gridFileDir, META_FILE);
		if(!gridMetadata.isFile())
			throw new Exception(String.format("%s is not found in dictionary",gridMetadata.getName()));
		//read parameters from meta data file, in xml
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(gridMetadata);
		doc.getDocumentElement().normalize();
		//read meta data from GridmapMatrix first
		NodeList metaNodes = doc.getElementsByTagName("GridmapMatrix");
		if(metaNodes.getLength()>1)
			throw new Exception("bad meta data file, more than one <GridmapMatrix> node");
		if(metaNodes.getLength()==0)
			throw new Exception("bad meta data file, no <GridmapMatrix> node");
		Element metaNode = (Element)metaNodes.item(0);
		double org_x = Double.parseDouble(metaNode.getElementsByTagName("matrix_org_x").item(0).getTextContent());
		double org_y = Double.parseDouble(metaNode.getElementsByTagName("matrix_org_y").item(0).getTextContent());
		double size_x = Double.parseDouble(metaNode.getElementsByTagName("matrix_size_x").item(0).getTextContent());
		double size_y = Double.parseDouble(metaNode.getElementsByTagName("matrix_size_y").item(0).getTextContent());
		
		double grid_size_x = Double.parseDouble(metaNode.getElementsByTagName("grid_size_x").item(0).getTextContent());
		double grid_size_y = Double.parseDouble(metaNode.getElementsByTagName("grid_size_y").item(0).getTextContent());
		double grid_res_x = Double.parseDouble(metaNode.getElementsByTagName("grid_res_x").item(0).getTextContent());
		double grid_res_y = Double.parseDouble(metaNode.getElementsByTagName("grid_res_y").item(0).getTextContent());		

		GridmapMatrix<T> matrix = new GridmapMatrix<T>(gridFileDir, org_x, org_y, size_x, size_y, new GridmapAttribute(grid_size_x, grid_size_y, grid_res_x, grid_res_y), emptyCell, readOnly);
		//read Gridmap index data
		Element indexNode = (Element)doc.getElementsByTagName("GridmapIndex").item(0);
		NodeList gridmapNodes = indexNode.getElementsByTagName("Gridmap");
		if(gridmapNodes.getLength()==0){
			System.err.println("no Gridmap node found");
		}
		for(int i=0; i<gridmapNodes.getLength(); i++){
			Element n = (Element)gridmapNodes.item(i);
			int x = Integer.parseInt(n.getElementsByTagName("x").item(0).getTextContent());
			int y = Integer.parseInt(n.getElementsByTagName("y").item(0).getTextContent());
			File gridFile = new File(gridFileDir ,n.getElementsByTagName("filename").item(0).getTextContent());
			matrix.addGridIndex(new Index2D(x, y),  gridFile);
		}
		return matrix;
	}
	
	/**
	 * make new GridmapMatrix in the specified directory with specified parameters
	 * @param gridFileDir
	 * @param org_x
	 * @param org_y
	 * @param size_x
	 * @param size_y
	 * @param attr: attribute of mini Gridmap
	 * @param emptyCell: an example of empty cell that is used when a new map is created 
	 * @return
	 * @throws Exception
	 */
	public static <T extends Cell> GridmapMatrix<T> makeGridmapMatrix(File gridFileDir, double org_x, double org_y, double size_x, double size_y, GridmapAttribute attr, T emptyCell) throws Exception{
		File metaFile = new File(gridFileDir, META_FILE);
		//if gridFileDir already exits and contains .meta file, new one should not be created
		if(metaFile.isFile()){
			throw new Exception(String.format("%s is already a valid Gridmap data warehouse with .meta file", gridFileDir));
		}else{
			//if make dir and file
			gridFileDir.mkdirs();
			metaFile.createNewFile();
		}
		GridmapMatrix<T> matrix = new GridmapMatrix<T>(gridFileDir, org_x, org_y, size_x, size_y, attr, emptyCell, false);
		return matrix;
	}
	
	private GridmapMatrix(File gridFileDir, double org_x, double org_y, double size_x, double size_y, GridmapAttribute attr, T emptyCell, boolean readOnly) {
		this.org_x = org_x;
		this.org_y = org_y;
		this.size_x = size_x;
		this.size_y = size_y;
		
		this.gridmapAttr = attr;
		
		this.rows = (int)Math.ceil(this.size_y / attr.size_y);
		this.cols = (int)Math.ceil(this.size_x / attr.size_x);
		this.res_x = attr.size_x;
		this.res_y = attr.size_y;
		this.size_x = this.cols * this.res_x;
		this.size_y = this.rows * this.res_y;
		
		this.gridFileDir = gridFileDir;
		this.cacheSize = DEFAULT_CACHE_SIZE;
		this.gridCache = new GridCache<T>(this.cacheSize, LOAD_FACTOR, this);
		this.gridIndex = new HashMap<Index2D, File>();
		this.readOnly = readOnly;
		
		this.emptyCell = emptyCell;
	}
	
	/**
	 * Write meta data and grid map index into metaFile
	 * @throws Exception
	 */
	private void writeMetaFile() throws Exception{
		File metaFile = new File(this.gridFileDir, META_FILE);
		metaFile.createNewFile();
		//fill metaFile
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.newDocument();
		Element root = doc.createElement("MetaData");doc.appendChild(root);
		//GridmapMatrix node
		Element metaNode = doc.createElement("GridmapMatrix");root.appendChild(metaNode);
		Element org_x_node = doc.createElement("matrix_org_x"); org_x_node.appendChild(doc.createTextNode(String.format("%.3f", org_x))); 
		metaNode.appendChild(org_x_node);
		Element org_y_node = doc.createElement("matrix_org_y"); org_y_node.appendChild(doc.createTextNode(String.format("%.3f", org_y))); 
		metaNode.appendChild(org_y_node);
		Element size_x_node = doc.createElement("matrix_size_x"); size_x_node.appendChild(doc.createTextNode(String.format("%.3f", size_x))); 
		metaNode.appendChild(size_x_node);
		Element size_y_node = doc.createElement("matrix_size_y"); size_y_node.appendChild(doc.createTextNode(String.format("%.3f", size_y))); 
		metaNode.appendChild(size_y_node);
		Element grid_size_x_node = doc.createElement("grid_size_x"); grid_size_x_node.appendChild(doc.createTextNode(String.format("%.3f", gridmapAttr.size_x))); 
		metaNode.appendChild(grid_size_x_node);
		Element grid_size_y_node = doc.createElement("grid_size_y"); grid_size_y_node.appendChild(doc.createTextNode(String.format("%.3f", gridmapAttr.size_y))); 
		metaNode.appendChild(grid_size_y_node);
		Element grid_res_x_node = doc.createElement("grid_res_x"); grid_res_x_node.appendChild(doc.createTextNode(String.format("%.3f", gridmapAttr.res_x))); 
		metaNode.appendChild(grid_res_x_node);
		Element grid_res_y_node = doc.createElement("grid_res_y"); grid_res_y_node.appendChild(doc.createTextNode(String.format("%.3f", gridmapAttr.res_y))); 
		metaNode.appendChild(grid_res_y_node);
		
		//GridmapIndex node
		Element indexNode = doc.createElement("GridmapIndex");root.appendChild(indexNode);
		for(Entry<Index2D, File> entry: this.gridIndex.entrySet()){
			Element n = doc.createElement("Gridmap");indexNode.appendChild(n);
			Attr a = doc.createAttribute("id"); a.setValue(entry.getKey().toString());n.setAttributeNode(a);
			Element x = doc.createElement("x"); x.appendChild(doc.createTextNode(String.format("%d", entry.getKey().row))); n.appendChild(x);
			Element y = doc.createElement("y"); y.appendChild(doc.createTextNode(String.format("%d", entry.getKey().col))); n.appendChild(y);
			Element filename = doc.createElement("filename"); filename.appendChild(doc.createTextNode(entry.getValue().getName())); n.appendChild(filename);
		}
		//write out
		TransformerFactory tf = TransformerFactory.newInstance();
		Transformer trans = tf.newTransformer();
		DOMSource source = new DOMSource(doc);
		StreamResult result = new StreamResult(metaFile);
		//StreamResult result = new StreamResult(System.out);
		trans.transform(source, result);
	}
	
	/**
	 * once finish and if not readOnly
	 * all the mini Gridmap in cache will be dumped to disk
	 * metadata.xml will be modified
	 */
	public void finish(){
		if(!this.readOnly){//if not readOnly, dump grids in the cache to the disk
			for(Entry<Index2D, Gridmap<T>> e : gridCache.entrySet()){
				File f = this.gridIndex.get(e.getKey());
				try{
					e.getValue().dumpToFile(f);
				}catch(Exception ee){
					ee.printStackTrace();
					System.err.println("dump file failed "+e.getKey());
				}
			}
		}
		gridCache.clear();
		try{
			if(!this.readOnly){
				System.out.println("write meta file");
				this.writeMetaFile();//if readOnly, then no need to change meta file
			}
		}catch(Exception e){
			e.printStackTrace();
			System.err.println("write meta file failed!");
		}
	}
	/**
	 * make a QuadGridmap based on the position and range of LIDAR
	 * @param x north
	 * @param y east
	 * @param range
	 * @return null if exception raised or position not in Matrix
	 */
	public QuadGridmap<T> makeQuadGridmap(double east, double north, double range){
		Index2D org = findIndex(east, north);
		if (org==null) return null;
		Index2D ul=null, ur=null, dl=null, dr = null;
		try{
			Gridmap<T> orgGridmap = this.getGridmap(org);
			System.out.printf("org(%.2f, %.2f), pos(%.2f, %.2f)\n" , orgGridmap.org_x, orgGridmap.org_y, east, north);
			if(east-range<orgGridmap.org_x && north+range>orgGridmap.org_y){//downRight
				dr=org; dl=dr.left(); ul=dl.up(); ur=ul.right();
			}else if(east-range<orgGridmap.org_x && north+range<=orgGridmap.org_y){//upRight
				ur=org; dr=ur.down(); dl=dr.left(); ul=dl.up();
			}else if(east-range>=orgGridmap.org_x && north+range>orgGridmap.org_y){//downLeft
				dl=org; dr=dl.right(); ul=dl.up(); ur=ul.right();
			}else{//upLeft
				ul=org; dl=ul.down(); dr=dl.right(); ur=dr.up();
			}
			QuadGridmap<T> quad = new QuadGridmap<T>(isValidIndex(ul) ? this.getGridmap(ul) : null, 
									isValidIndex(ur) ? this.getGridmap(ur) : null,
									isValidIndex(dl) ? this.getGridmap(dl) : null,
									isValidIndex(dr) ? this.getGridmap(dr) : null);
			if(!quad.isComplete()){
				throw new Exception(String.format("GridmapMatrix: cannot complete quadMap of %s(ur), %s(dl), %s(dr) for %s\n", ul, ur, dl, dr, org));
			}
			System.out.printf("GridmapMatrix: find %s(ul), %s(ur), %s(dl), %s(dr) for %s\n", ul, ur, dl, dr, org);
			return quad;
		}catch(Exception e){
			e.printStackTrace();
			return null;
		}finally{
		}
	}
	private boolean isValidIndex(Index2D index){
		return index.row>=0 && index.row<rows && index.col>=0 && index.col<cols;
	}
	/**
	 * find Gridmap index from vehicle position
	 * @param x-east, position corresponds to cols
	 * @param y-north, position corresponds to rows
	 * @return null if position does not correspond to any cell in Matrix
	 */
	private Index2D findIndex(double east, double north){
		int col = (int)Math.floor((east-org_x)/res_x);
		int row = (int)Math.floor((org_y-north)/res_y);
		if(col<0 || row<0 || col>=this.cols || row>=this.rows){
			System.err.println("bad position leads to exceeding GridmapMatrix index");
			System.err.printf("(%.3f, %.3f) -> (%d, %d)\n",east, north, col, row);
			return null;
		}
		return new Index2D(row, col);
	}

	/**
	 * get Gridmap from this matrix
	 * 1. get from gridCache, if in cache, return
	 * 2. if not in cache, search in gridIndx, if in, load Gridmap from disc
	 * 3. if not in index, create a new dump file 
	 * @param index
	 * @return
	 * @throws Exception
	 */
	public Gridmap<T> getGridmap(Index2D index) throws Exception{
		Gridmap<T> gridmap = this.gridCache.get(index);
		if(gridmap==null){//miss in cahce
			File newFile = this.gridIndex.get(index);
			if(newFile==null){//miss on disk
				if(readOnly){//if read only, just return null
					return null;
				}else{//create a new Gridmap
					System.out.printf("start making new gridmap for %s\n", index);
					gridmap = this.makeGridMap(index);
					this.gridIndex.put(index, this.getDumpFileFromIndex(index));
					System.out.printf("made new gridmap for %s\n", index);	
				}
			}else{//load from disk
				gridmap = Gridmap.loadFromFile(newFile, this.gridmapAttr);
				System.out.printf("loading Gridmap for %s\n", index);
			}
			this.gridCache.put(index, gridmap);
		}else{
			//System.out.printf("use cache for %s\n", index);
		}
		return gridmap;
	}
	
	/**
	 * make Gridmap in memory
	 * it should only be called when Gridmap cannot be found on disc
	 * @param index
	 * @return
	 */
	private Gridmap<T> makeGridMap(Index2D index){
		double org_x = this.org_x + this.res_x * index.col;
		double org_y = this.org_y - this.res_y * index.row;
		//debug
		//System.out.printf("generating new Gridmap for %s\n", index);
		return new Gridmap<T>(org_x, org_y, this.gridmapAttr, this.emptyCell);
	}
	
	/**
	 * add grid file name and its index to gridIndex
	 * @param index
	 * @param newGridFile
	 */
	private void addGridIndex(Index2D index, File newGridFile){
		this.gridIndex.put(index, newGridFile);
	}
	
	/**
	 * generate the dump file name based on index 
	 * @param index
	 * @return
	 */
	private File getDumpFileFromIndex(Index2D index){
		return new File(this.gridFileDir, index.toString());
	}

	/**given the image size and resolution, down sample the grid map and save as image
	 * elements in aggRowNum*aggColNum will aggregate to be a single pixel 
	 * @param aggRowNum number of rows to aggregate
	 * @param aggColNum number of cols to aggregate
	 */
	public void drawGridmapImage(int aggRowNum, int aggColNum, File imgFile) throws Exception{
		if(!new File(imgFile.getParent()).isDirectory()) 
			throw new Exception(String.format("the image file %s cannot be created, check path", imgFile));
		//calculate row and col of new image in pixels
		int height = (int)Math.ceil(gridmapAttr.rows/aggRowNum);
		int width = (int)Math.ceil(gridmapAttr.cols/aggColNum);
		BufferedImage image = new BufferedImage(width*cols, height*rows, BufferedImage.TYPE_INT_ARGB);
		
		int cnt=0;
		//go through each grid map, and set image pixels
		for(int i=0; i<rows; i++){
			for(int j=0; j<cols; j++){
				//load from disc
				//if not on disc, skip, the region will be black
				File f = this.gridIndex.get(new Index2D(i, j));
				if(f==null){
					for(int k=0; k<height; k++){
						for(int l=0; l<width; l++){
							image.setRGB(l+j*width, k+i*height, 0xffff<<24); //fill black
						}
					}
					continue;
				}
				System.out.println("draw grid map "+f);
				Gridmap<T> grid = Gridmap.loadFromFile(f, this.gridmapAttr);
				
				//get aggregated value matrix from gridmap
				//format: double[height][width]
				double[][] values = grid.getCellValues();
				//cnt = this.drawHelper(image, values1, values2, j*width, i*height, width,height);
				cnt = this.drawHelper(image, values, j*width, i*height, width,height);
				
				System.out.printf("(%d,%d) window, %d cells are drawn\n", width, height, cnt);
				cnt=0;
			}
		}
		ImageIO.write(image, "png", imgFile);
	}
	
	public void drawMultiGridmapImage(int row1, int row2, int col1, int col2, File imgFile, double occupyProb) throws Exception{
		if(!new File(imgFile.getParent()).isDirectory()) 
			throw new Exception(String.format("the image file %s cannot be created, check path", imgFile));
		int numRows = row2 - row1 +1;
		int numCols = col2 - col1 +1;
		//calculate row and col of new image in pixels
		BufferedImage image = new BufferedImage(gridmapAttr.cols * numCols, gridmapAttr.rows * numRows, BufferedImage.TYPE_INT_ARGB);
		
		for(int row=row1; row<=row2; row++){
			for(int col=col1; col<=col2; col++){
				File f = this.gridIndex.get(new Index2D(row, col));
				if(f==null){
					System.out.printf("no map for %d_%d\n", row, col);
					continue;
				}
				System.out.println("draw grid map "+f);
				Gridmap<T> grid = Gridmap.loadFromFile(f, this.gridmapAttr);
				double[][] values = grid.getCellValues();
				//go through each grid map, and set image pixels
				int orgRow=(row-row1)*gridmapAttr.rows;
				int orgCol=(col-col1)*gridmapAttr.cols;
				for(int i=0; i<gridmapAttr.rows; i++){
					for(int j=0; j<gridmapAttr.cols; j++){
						//load from disc
						//image.setRGB(l+j*width, k+i*height, 0xffff<<24); //fill black

						//format: double[height][width]
						double v = values[i][j];
						if(v<occupyProb) image.setRGB(j+orgCol, i+orgRow, 0xffff<<24);
						else image.setRGB(j+orgCol, i+orgRow, ((int)0xffff)<<0 | ((int)0xffff)<<8 | ((int)0xffff)<<16 | ((int)0xffff)<<24);
					}
				}
			}
		}
		
		ImageIO.write(image, "png", imgFile);
	}
	
	/**
	 * draw a mini Gridmap with index (row, col) to imgFile
	 * only draw the grid where occupy value > occupy_prob
	 * @param row
	 * @param col
	 * @param imgFile
	 * @param occupyProb
	 * @throws Exception
	 */
	public void drawMinGridmapImage(int row, int col, File imgFile, double occupyProb) throws Exception{
		if(!new File(imgFile.getParent()).isDirectory()) 
			throw new Exception(String.format("the image file %s cannot be created, check path", imgFile));
		//calculate row and col of new image in pixels
		BufferedImage image = new BufferedImage(gridmapAttr.cols, gridmapAttr.rows, BufferedImage.TYPE_INT_ARGB);
		File f = this.gridIndex.get(new Index2D(row, col));
		if(f==null){
			return;
		}
		System.out.println("draw grid map "+f);
		Gridmap<T> grid = Gridmap.loadFromFile(f, this.gridmapAttr);
		double[][] values = grid.getCellValues();
		//go through each grid map, and set image pixels
		for(int i=0; i<gridmapAttr.rows; i++){
			for(int j=0; j<gridmapAttr.cols; j++){
				//load from disc
				//image.setRGB(l+j*width, k+i*height, 0xffff<<24); //fill black

				//format: double[height][width]
				double v = values[i][j];
				if(v<occupyProb) image.setRGB(j, i, 0xffff<<24);
				else image.setRGB(j, i, ((int)0xffff)<<0 | ((int)0xffff)<<8 | ((int)0xffff)<<16 | ((int)0xffff)<<24);
			}
		}
		ImageIO.write(image, "png", imgFile);
	}

	private int drawHelper(BufferedImage image, double[][] pixels, int xoffset, int yoffset, int dx, int dy){
		int cnt=0;
		for(int k=0; k<dy; k++){
			for(int l=0; l<dx; l++){
				//double value = values[k][j]==0 ? 0 : 255 - (values[k][j]-this.occupy_prob) / (1-this.occupy_prob) *255;
				double value = pixels[k][l];
				if(value<0.9){
					image.setRGB(l+xoffset, k+yoffset, 0xffff<<24);
					continue;
				}else{
					//value is within 255, showing blue in diff alpha value, ARGB int: A<<24|R<<16|G<<8|B<<0
					int v = (int)((value-0.8)*255);
					v=255;
					image.setRGB(l+xoffset, k+yoffset, ((int)v)<<0 | ((int)v)<<8 | ((int)v)<<16 | 0xffff<<24);
					//image.setRGB(l+xoffset, k+yoffset, 0xffff);
					cnt++;
				}
			}
		}
		return cnt;
	}
	
	/**
	 * print gridIndex out
	 * index - file name
	 * @return
	 */
	private String gridmapIndexString(){
		StringBuilder sb = new StringBuilder();
		for(Entry<Index2D, File> e: this.gridIndex.entrySet()){
			sb.append(e.getKey());
			sb.append("---");
			sb.append(e.getValue().getName());
			sb.append('\n');
		}
		return sb.toString();
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("orgin: (");sb.append(this.org_x);sb.append(this.org_y);sb.append(")\n");
		sb.append("matrix_size_x");sb.append(this.size_x);
		sb.append('\t');
		sb.append("matrix_size_y");sb.append(this.size_y);
		sb.append('\n');
		sb.append("matrix_res_x");sb.append(this.res_x);
		sb.append('\t');
		sb.append("matrix_res_y");sb.append(this.res_y);
		sb.append('\n');
		sb.append("matrix_rows");sb.append(this.rows);
		sb.append('\t');
		sb.append("matrix_cols");sb.append(this.cols);
		sb.append('\n');
		sb.append("Gridmap:\n");
		sb.append(gridmapAttr);
		sb.append('\n');
		return sb.toString();
	}

}

class Index2D implements Serializable{
	int row;
	int col;
	public Index2D(int row, int col) {
		this.row = row;
		this.col = col;
	}
	@Override
	public boolean equals(Object other) {
		if(!(other instanceof Index2D)) return false;
		return row==((Index2D)other).row && col==((Index2D)other).col;
	}
	@Override
	public int hashCode() {
		final int prime=31;
		int res =1;
		return prime*res + row + col;
	}
	@Override
	public String toString() {
		return String.format("%d_%d",row,col);
	}
	public Index2D left(){
		return new Index2D(row, col-1);
	}
	public Index2D right(){
		return new Index2D(row, col+1);
	}
	public Index2D up(){
		return new Index2D(row-1, col);
	}
	public Index2D down(){
		return new Index2D(row+1, col);
	}
	public Index2D leftUp(){
		return new Index2D(row-1, col-1);
	}
	public Index2D rightUp(){
		return new Index2D(row-1, col+1);
	}
	public Index2D leftDown(){
		return new Index2D(row+1, col-1);
	}
	public Index2D rightDown(){
		return new Index2D(row+1, col+1);
	}
}


class GridCache <T extends Cell> extends LinkedHashMap<Index2D, Gridmap<T>> {
	private int cacheSize;
	private GridmapMatrix<T> parent;

	public GridCache(int cacheSize, float loadFactor, GridmapMatrix<T> gmm) {
		super((int)(Math.ceil(cacheSize/loadFactor)), loadFactor, true);
		this.cacheSize = cacheSize;
		this.parent = gmm;
	}
	
	@Override
	protected boolean removeEldestEntry(Map.Entry<Index2D,Gridmap<T>> eldest) {
		System.out.printf("Cache size is %d\n", this.size());
		if(this.size()+1 > this.cacheSize){
			try{
				//this guarantee the dump file name is identical to the gridIndex
				if(!this.parent.readOnly){
					System.out.printf("start dump %s to disk\n", eldest.getKey());
					File dumpFile = this.parent.gridIndex.get(eldest.getKey());
					eldest.getValue().dumpToFile(dumpFile);
					System.out.printf("end dump %s to disk\n", eldest.getKey());
				}
			}catch(Exception e){
				//notify an error listener or write to error log
			}
			return true;
		}else{
			return false;
		}
	};
};
