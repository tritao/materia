package cadkit.sheetmetal;

/** A permanent connection between two straight boundary segments, with angle in radians. */
class Bend {
	public final id:Int;
	public final wallA:Int;
	public final edgeA:WallEdge;
	public final wallB:Int;
	public final edgeB:WallEdge;
	public final angle:Float;
	public final radius:Float;
	public function new(id:Int, wallA:Int, edgeA:WallEdge, wallB:Int, edgeB:WallEdge, angle:Float, radius:Float) {
		this.id = id;
		this.wallA = wallA;
		this.edgeA = edgeA;
		this.wallB = wallB;
		this.edgeB = edgeB;
		this.angle = angle;
		this.radius = radius;
	}
}
