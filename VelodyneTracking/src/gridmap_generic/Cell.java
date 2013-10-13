package gridmap_generic;

import java.io.Serializable;

public interface Cell extends Serializable{
	public double getValue();
	public void update(double value);
	public void reset();
	public Cell cloneCell();
}
