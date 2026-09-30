package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import robotkit.runtime.Simulation;
import stockkit.CutMotion;
import stockkit.CutMove;
import stockkit.Stock;
import stockkit.StockColoring;
import stockkit.StockLattice;
import stockkit.StockPreview;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.tool.Tool;

/**
 * The stock on a simulated machine, cut by the tool as the machine actually moves. Each call to
 * `follow` reads where the tool tip is now and cuts the straight segment from where it was, so the
 * stock records the motion the simulation produced, following error included, not the program.
 *
 * Coordinates are metres in the stock part's frame shifted to its preview centre, the frame its scene
 * geometry uses, so meshes go to the scene unchanged. The tool part's origin must be its tip and its
 * +Z the tool axis, which must stay along the stock's +Z (three-axis machining).
 */
class MachiningStock {
	/** Grey of stock the tool has not touched, and of cut surfaces, as RGBA. */
	static inline final UNTOUCHED = 0xB4B9BFFF;
	static inline final CUT = 0xE3E6EAFF;
	/** Tool motion shorter than this since the last cut is left for the next one, in metres. */
	static inline final MIN_SEGMENT = 1e-6;

	public final stock:Stock;
	final preview:StockPreview;
	final tool:Tool;
	final simulation:Simulation;
	final robotIndex:Int;
	final stockLink:Int;
	final toolLink:Int;
	final center:Array<Float>;
	var last:Null<Point3> = null;
	var changed = true;
	/** Volume removed so far, in cubic metres. */
	public var removed(default, null):Float = 0.0;
	/** Segments that rapid moves cut, and segments whose shank or holder met the stock. */
	public var rapidContacts(default, null):Int = 0;
	public var collisions(default, null):Int = 0;

	/**
	 * `minimum` and `maximum` bound the stock in its part's frame and `center` is that part's preview
	 * centre, in metres; the stock is that box. `spacing` is the stock's ray spacing.
	 */
	public function new(tool:Tool, minimum:Array<Float>, maximum:Array<Float>, center:Array<Float>, spacing:Float,
			simulation:Simulation, robotIndex:Int, stockLink:Int, toolLink:Int) {
		this.tool = tool;
		this.simulation = simulation;
		this.robotIndex = robotIndex;
		this.stockLink = stockLink;
		this.toolLink = toolLink;
		this.center = center.copy();
		var low = [for (axis in 0...3) minimum[axis] - center[axis]];
		var high = [for (axis in 0...3) maximum[axis] - center[axis]];
		var lattice = StockLattice.covering(low[0], low[1], low[2], high[0], high[1], high[2], spacing);
		stock = Stock.box(lattice, low[0], low[1], low[2], high[0], high[1], high[2]);
		preview = new StockPreview(stock, BySource(_ -> CUT, UNTOUCHED));
		var axis = toolAxis();
		if (axis[2] < 1 - 1e-6)
			throw "The machining tool must point along the stock's +Z";
	}

	/**
	 * Cuts from where the tool tip was at the last call to where it is now. `kind`, `opIndex` and
	 * `provenance` describe the program move under way, for diagnostics and picking.
	 */
	public function follow(kind:MoveKind, opIndex:Int, provenance:Provenance):Void {
		var tip = toolTip();
		var start = last;
		last = tip;
		if (start == null) return;
		var dx = tip.x - start.x, dy = tip.y - start.y, dz = tip.z - start.z;
		if (Math.sqrt(dx * dx + dy * dy + dz * dz) < MIN_SEGMENT) {
			last = start;
			return;
		}
		var report = stock.cut([new CutMove(tool, CutMotion.Path(PathGeometry.Line(start, tip)), kind, opIndex, provenance)]);
		var volume = report.removedVolume();
		if (volume > 0) changed = true;
		removed += volume;
		rapidContacts += report.rapidContacts().length;
		collisions += report.collisions().length;
	}

	/** Whether the stock changed since the last `geometry()`. */
	public function hasChanged():Bool return changed;

	/** The stock as it is now: only chunks that changed are contoured again. */
	public function geometry():GeometryData {
		preview.update();
		changed = false;
		var vertices = 0, triangles = 0;
		for (mesh in preview.meshes) if (mesh != null) {
			vertices += mesh.vertexCount;
			triangles += mesh.triangleCount;
		}
		var positions = Bytes.alloc(vertices * 12), normals = Bytes.alloc(vertices * 12);
		var colors = Bytes.alloc(vertices * 4), indices = Bytes.alloc(triangles * 12);
		var vertexBase = 0, indexBase = 0;
		for (mesh in preview.meshes) if (mesh != null) {
			positions.blit(vertexBase * 12, mesh.positions, 0, mesh.vertexCount * 12);
			normals.blit(vertexBase * 12, mesh.normals, 0, mesh.vertexCount * 12);
			colors.blit(vertexBase * 4, mesh.colors, 0, mesh.vertexCount * 4);
			for (index in 0...mesh.triangleCount * 3)
				indices.setInt32((indexBase + index) * 4, mesh.indices.getInt32(index * 4) + vertexBase);
			vertexBase += mesh.vertexCount;
			indexBase += mesh.triangleCount * 3;
		}
		var lattice = stock.lattice;
		var geometry = new GeometryData();
		geometry.addStream(1, 2, positions, vertices, 12);
		geometry.addStream(2, 2, normals, vertices, 12);
		geometry.addStream(6, 4, colors, vertices, 4);
		geometry.setIndexBuffer(indices, triangles * 3);
		geometry.setBounds(lattice.x(0), lattice.y(0), lattice.z(0), lattice.x(lattice.countX - 1),
			lattice.y(lattice.countY - 1), lattice.z(lattice.countZ - 1));
		return geometry;
	}

	/** The tool tip in the stock's frame. */
	function toolTip():Point3 {
		var stockPose = simulation.linkPose(robotIndex, stockLink), toolPose = simulation.linkPose(robotIndex, toolLink);
		var local = inverseRotate(stockPose.rotation, [for (axis in 0...3) toolPose.position[axis] - stockPose.position[axis]]);
		return new Point3(local[0] - center[0], local[1] - center[1], local[2] - center[2]);
	}

	/** The tool's +Z in the stock's frame. */
	function toolAxis():Array<Float> {
		var stockPose = simulation.linkPose(robotIndex, stockLink), toolPose = simulation.linkPose(robotIndex, toolLink);
		return inverseRotate(stockPose.rotation, rotate(toolPose.rotation, [0.0, 0.0, 1.0]));
	}

	static function rotate(q:Array<Float>, v:Array<Float>):Array<Float> {
		var x = q[0], y = q[1], z = q[2], w = q[3];
		var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
		return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
	}

	static function inverseRotate(q:Array<Float>, v:Array<Float>):Array<Float>
		return rotate([-q[0], -q[1], -q[2], q[3]], v);

	public function dispose():Void stock.dispose();
}
