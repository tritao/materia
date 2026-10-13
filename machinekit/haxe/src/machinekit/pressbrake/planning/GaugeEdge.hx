package machinekit.pressbrake.planning;

import cadkit.sheetmetal.WallEdge;

/** Authored wall boundary used for registration; native edge numbering is never a datum. */
class GaugeEdge {
	public final wall:Int;
	public final edge:WallEdge;
	public function new(wall:Int, edge:WallEdge) {
		if (wall <= 0 || edge == null) throw "Gauge edge needs a wall identity and authored boundary";
		this.wall = wall; this.edge = edge;
	}
}
