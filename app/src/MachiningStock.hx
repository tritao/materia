package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import robotkit.runtime.Simulation;
import stockkit.CutMotion;
import stockkit.CutMove;
import stockkit.Stock;
import stockkit.StockColoring;
import stockkit.StockLattice;
import stockkit.StockMesh;
import stockkit.StockPreviewWorker;
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
 * geometry uses, so meshes go to the scene unchanged. The spindle part's origin is the gauge line and
 * its +Z runs up the spindle axis, which must stay along the stock's +Z (three-axis machining); the
 * loaded tool's tip hangs its length below the gauge line.
 */
class MachiningStock {
	/** Grey of stock the tool has not touched, and of cut surfaces, as RGBA. */
	static inline final UNTOUCHED = 0xB4B9BFFF;
	static inline final CUT = 0xE3E6EAFF;
	/** Against a target: surfaces within tolerance, stock left on, and cuts into the part. */
	static inline final ON_TARGET = 0x6FB66FFF;
	static inline final LEFTOVER = 0xE0C040FF;
	static inline final GOUGE = 0xD04040FF;
	/** Deviation below this counts as on target, in metres. */
	public static inline final TOLERANCE = 0.00002;
	/**
	 * Contact below this volume (cubic metres, a thousandth of a cubic millimetre) is the ray stock's
	 * numerical grazing where a move ends at a surface, not a rapid or holder entering the stock.
	 */
	static inline final CONTACT_VOLUME = 1e-12;
	/** Tool motion shorter than this since the last cut is left for the next one, in metres. */
	static inline final MIN_SEGMENT = 1e-6;

	public final stock:Stock;
	/** The finished part on the same lattice, when the job has one. */
	public final target:Null<Stock>;
	/** Contours the stock on its own thread, from snapshots, as it is cut. */
	final preview:StockPreviewWorker;
	/** Each chunk's latest mesh, row by row, as refreshes have delivered them. */
	var chunkMeshes:Array<Null<StockMesh>> = [];
	/** The tool in the spindle; nothing cuts until one is loaded. */
	var tool:Null<Tool> = null;
	final simulation:Simulation;
	final stockPart:AssemblyRobot.AssemblyPart;
	final spindlePart:AssemblyRobot.AssemblyPart;
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
	 * centre, in metres. The stock is `stockMesh` (in the part's frame, metres) when given, else that
	 * box. `spacing` is the stock's ray spacing.
	 */
	public function new(minimum:Array<Float>, maximum:Array<Float>, center:Array<Float>, spacing:Float,
			simulation:Simulation, stockPart:AssemblyRobot.AssemblyPart, spindlePart:AssemblyRobot.AssemblyPart,
			?targetMesh:{positions:Array<Float>, indices:Array<Int>}, ?stockMesh:{positions:Array<Float>, indices:Array<Int>}) {
		this.simulation = simulation;
		this.stockPart = stockPart;
		this.spindlePart = spindlePart;
		this.center = center.copy();
		var low = [for (axis in 0...3) minimum[axis] - center[axis]];
		var high = [for (axis in 0...3) maximum[axis] - center[axis]];
		var lattice = StockLattice.covering(low[0], low[1], low[2], high[0], high[1], high[2], spacing);
		// The raw stock is the stock part's own solid when its mesh is known, else its bounding box.
		stock = stockMesh == null ? Stock.box(lattice, low[0], low[1], low[2], high[0], high[1], high[2])
			: Stock.fromTriangles(lattice, [for (index in 0...stockMesh.positions.length)
				stockMesh.positions[index] - center[index % 3]], stockMesh.indices);
		if (targetMesh == null) {
			target = null;
			preview = new StockPreviewWorker(stock, BySource(_ -> CUT, UNTOUCHED));
		} else {
			// The target is given in the stock part's frame; the stock lives shifted to its centre.
			var positions = [for (index in 0...targetMesh.positions.length)
				targetMesh.positions[index] - center[index % 3]];
			var finished = Stock.fromTriangles(lattice, positions, targetMesh.indices);
			target = finished;
			preview = new StockPreviewWorker(stock, ByDeviation(finished, TOLERANCE, ON_TARGET, LEFTOVER, GOUGE));
		}
		var axis = spindleAxis();
		if (axis[2] < 1 - 1e-6)
			throw "The machining spindle must point along the stock's +Z";
	}

	/** Puts `next` in the spindle. The tip jumps with the tool's length, so cutting starts afresh. */
	public function load(next:Null<Tool>):Void {
		tool = next;
		last = null;
	}

	/**
	 * Cuts from where the tool tip was at the last call to where it is now. `kind`, `opIndex` and
	 * `provenance` describe the program move under way, for diagnostics and picking.
	 */
	public function follow(kind:MoveKind, opIndex:Int, provenance:Provenance):Void {
		var loaded = tool;
		if (loaded == null) return;
		var tip = toolTip(loaded);
		var start = last;
		last = tip;
		if (start == null) return;
		var dx = tip.x - start.x, dy = tip.y - start.y, dz = tip.z - start.z;
		if (Math.sqrt(dx * dx + dy * dy + dz * dz) < MIN_SEGMENT) {
			last = start;
			return;
		}
		var report = stock.cut([new CutMove(loaded, CutMotion.Path(PathGeometry.Line(start, tip)), kind, opIndex, provenance)]);
		var volume = report.removedVolume();
		if (volume > 0) changed = true;
		removed += volume;
		rapidContacts += report.rapidContacts(CONTACT_VOLUME).length;
		collisions += report.collisions(CONTACT_VOLUME).length;
	}

	/** Whether the stock changed since the last refresh started. */
	public function hasChanged():Bool return changed;

	/** Starts contouring the stock as it is now, unless a refresh is under way; returns whether it started. */
	public function refreshPreview():Bool {
		if (!preview.refresh()) return false;
		changed = false;
		return true;
	}

	/**
	 * The chunks finished refreshes rebuilt, as geometry by chunk index, or null when none finished.
	 * `previewChunks` is how many chunks the stock shows as.
	 */
	public function takePreview():Null<Map<Int, GeometryData>> {
		var updates = preview.take();
		if (updates.length == 0) return null;
		var changedChunks:Map<Int, GeometryData> = new Map();
		for (update in updates) {
			if (chunkMeshes.length != update.chunksX * update.chunksY)
				chunkMeshes = [for (_ in 0...update.chunksX * update.chunksY) null];
			for (k in 0...update.chunks.length) {
				chunkMeshes[update.chunks[k]] = update.meshes[k];
				changedChunks.set(update.chunks[k], meshGeometry([update.meshes[k]]));
			}
		}
		return changedChunks;
	}

	public function previewChunks():Int return chunkMeshes.length;

	/** The whole stock as it is now, as one mesh: waits for its contouring. */
	public function geometry():GeometryData {
		preview.wait();
		refreshPreview();
		preview.wait();
		takePreview();
		return meshGeometry([for (mesh in chunkMeshes) if (mesh != null) mesh]);
	}

	function meshGeometry(meshes:Array<StockMesh>):GeometryData {
		var vertices = 0, triangles = 0;
		for (mesh in meshes) {
			vertices += mesh.vertexCount;
			triangles += mesh.triangleCount;
		}
		var positions = Bytes.alloc(vertices * 12), normals = Bytes.alloc(vertices * 12);
		var colors = Bytes.alloc(vertices * 4), indices = Bytes.alloc(triangles * 12);
		var vertexBase = 0, indexBase = 0;
		for (mesh in meshes) {
			positions.blit(vertexBase * 12, mesh.positions, 0, mesh.vertexCount * 12);
			normals.blit(vertexBase * 12, mesh.normals, 0, mesh.vertexCount * 12);
			colors.blit(vertexBase * 4, mesh.colors, 0, mesh.vertexCount * 4);
			if (vertexBase == 0) indices.blit(indexBase * 4, mesh.indices, 0, mesh.triangleCount * 12);
			else for (index in 0...mesh.triangleCount * 3)
				indices.setInt32((indexBase + index) * 4, mesh.indices.getInt32(index * 4) + vertexBase);
			vertexBase += mesh.vertexCount;
			indexBase += mesh.triangleCount * 3;
		}
		var geometry = new GeometryData();
		geometry.addStream(1, 2, positions, vertices, 12);
		geometry.addStream(2, 2, normals, vertices, 12);
		geometry.addStream(6, 4, colors, vertices, 4);
		geometry.setIndexBuffer(indices, triangles * 3);
		// Bounds of the meshes' own vertices, so a chunk culls as itself.
		if (vertices > 0) {
			var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (vertex in 0...vertices)
				for (axis in 0...3) {
					var value = positions.getFloat(vertex * 12 + axis * 4);
					if (value < low[axis]) low[axis] = value;
					if (value > high[axis]) high[axis] = value;
				}
			geometry.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
		}
		return geometry;
	}

	/** The tip of `loaded`, its length below the gauge line, in the stock's frame. */
	function toolTip(loaded:Tool):Point3 {
		var stockPose = AssemblyRobot.partPose(simulation, stockPart), spindlePose = AssemblyRobot.partPose(simulation, spindlePart);
		var down = rotate(spindlePose.rotation, [0.0, 0.0, -loaded.length]);
		var local = inverseRotate(stockPose.rotation,
			[for (axis in 0...3) spindlePose.position[axis] + down[axis] - stockPose.position[axis]]);
		return new Point3(local[0] - center[0], local[1] - center[1], local[2] - center[2]);
	}

	/** The spindle's +Z in the stock's frame. */
	function spindleAxis():Array<Float> {
		var stockPose = AssemblyRobot.partPose(simulation, stockPart), spindlePose = AssemblyRobot.partPose(simulation, spindlePart);
		return inverseRotate(stockPose.rotation, rotate(spindlePose.rotation, [0.0, 0.0, 1.0]));
	}

	static function rotate(q:Array<Float>, v:Array<Float>):Array<Float> {
		var x = q[0], y = q[1], z = q[2], w = q[3];
		var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
		return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
	}

	static function inverseRotate(q:Array<Float>, v:Array<Float>):Array<Float>
		return rotate([-q[0], -q[1], -q[2], q[3]], v);

	/**
	 * How the stock compares with the target: stock left on the part and material cut from it, in
	 * cubic metres, taking each from the grid that sees the most of it (Z sees floors, X and Y walls).
	 */
	public function deviation():{leftover:Float, gouge:Float} {
		var finished = target;
		if (finished == null) throw "The machining job has no target part";
		var leftover = 0.0, gouge = 0.0;
		for (comparison in stock.compareAll(finished)) {
			leftover = Math.max(leftover, comparison.leftoverVolume());
			gouge = Math.max(gouge, comparison.gougeVolume());
		}
		return {leftover: leftover, gouge: gouge};
	}

	public function dispose():Void {
		// The worker reads the target while it contours: stop it first.
		preview.dispose();
		stock.dispose();
		if (target != null) target.dispose();
	}
}
