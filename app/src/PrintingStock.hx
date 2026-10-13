package app;

import nativekit.scene.GeometryData;
import stockkit.CutMove;
import stockkit.CutMotion;
import stockkit.Stock;
import stockkit.StockColoring;
import stockkit.StockLattice;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.PrintFeature;
import toolpathkit.path.Provenance;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;

/**
 * The material a printer has laid, built up as its nozzle actually moves. Each call to `follow` reads where the nozzle
 * tip is now and how much filament the extruder has fed, and lays the stretch from where the tip was: a bead whose
 * section is the filament that was fed over the distance the tip travelled, as a flat-sided rectangle of the layer's height
 * with the width that holds that volume (`section`). Slip, a flow the hotend cannot melt and the pressure-advance
 * lead therefore show in what is deposited, not in what was programmed. Filament pulled back (a retraction) is owed
 * before more is laid.
 *
 * Coordinates are metres in the frame of the part that shows the print, shifted to its preview centre, the frame its
 * scene geometry uses. The nozzle's axis stays along the stock's +Z.
 */
class PrintingStock {
	/** Surfaces by what laid them, as RGBA; the colour of a stretch with no feature is `OTHER`. */
	static inline final EXTERNAL = 0xE8913AFF;
	static inline final PERIMETER = 0xD17A2AFF;
	static inline final INFILL = 0x9AA3ABFF;
	static inline final SOLID = 0x7F9FBFFF;
	static inline final TOP = 0xC8D6E5FF;
	static inline final SKIRT = 0x6E7378FF;
	static inline final OTHER = 0xB4B9BFFF;
	/** The tip moved less than this since the last call (2 mm/s over a tick of 10 ms) and stands still, in metres. */
	public static inline final STILL = 2e-5;
	/** Travel over which the section of a bead follows the filament fed, in metres. */
	public static inline final SMOOTHING = 0.0015;
	/** A bead's width and height are quantised to this in the cache of tools, in metres. */
	static inline final QUANTUM = 1e-5;
	/** A bead takes up at most this fraction of its volume more (or less) than it was fed, to make up a shortfall. */
	static inline final SQUEEZE = 0.5;
	/** The shortfall that is carried, at most, in cubic metres: more than a bead spilling out of nothing can be laid is lost. */
	static inline final LIMIT = 2e-9;
	/** How much wider than tall a bead is at least: a section under this squareness is laid as a square widened by it. */
	static inline final MIN_ASPECT = 1.05;

	public final stock:Stock;
	final meshes:StockMeshes;
	final filamentArea:Float;
	/** The feature a G-code line is printed as, as the program labels it. */
	final featureOfLine:Int->Null<PrintFeature>;
	final tools = new Map<String, Tool>();
	var nextTool = 1;
	var last:Null<Point3> = null;
	var lastFed = Math.NaN;
	/** The melt's lag behind the filament, seconds, and the filament pushed and not yet out of the nozzle, metres (negative after a retraction). */
	final meltLag:Float;
	var stored = 0.0;
	/** Filament fed and not yet laid, and pulled back and not yet pushed out again, in metres. */
	var pending = 0.0;
	var owed = 0.0;
	/** Filament pushed while the tip stood, and the filament's volume per distance as the section follows it. */
	var standing = 0.0;
	var density = 0.0;
	/** Volume fed and not laid, in cubic metres: negative when beads have laid more than they were fed. */
	var shortfall = 0.0;
	/** Volume laid so far, in cubic metres, and the beads it was laid in. */
	public var deposited(default, null):Float = 0.0;
	public var beads(default, null):Int = 0;

	/**
	 * `minimum` and `maximum` bound what can be laid in the part's frame and `center` is the part's preview centre,
	 * in metres. `spacing` is the stock's ray spacing; `filamentDiameter` is the filament's, in metres.
	 */
	public function new(minimum:Array<Float>, maximum:Array<Float>, center:Array<Float>, spacing:Float,
			filamentDiameter:Float, meltLag:Float, featureOfLine:Int->Null<PrintFeature>) {
		this.filamentArea = Math.PI * filamentDiameter * filamentDiameter / 4.0;
		this.meltLag = meltLag;
		this.featureOfLine = featureOfLine;
		var low = [for (axis in 0...3) minimum[axis] - center[axis]];
		var high = [for (axis in 0...3) maximum[axis] - center[axis]];
		var lattice = StockLattice.covering(low[0], low[1], low[2], high[0], high[1], high[2], spacing);
		stock = Stock.empty(lattice);
		meshes = new StockMeshes(stock, BySource(move -> colorOf(featureOfLine(move.provenance.line)), OTHER));
		this.center = center.copy();
	}

	/** The preview centre the stock's frame is shifted to, in metres: add it back for machine coordinates. */
	public final center:Array<Float>;

	static function colorOf(feature:Null<PrintFeature>):Int {
		if (feature == null) return OTHER;
		return switch feature {
			case ExternalPerimeter: EXTERNAL;
			case Perimeter: PERIMETER;
			case Infill: INFILL;
			case SolidInfill: SOLID;
			case TopSolidInfill: TOP;
			case Skirt: SKIRT;
			case _: OTHER;
		};
	}

	/**
	 * The bead of section `area` and height `height`: a flat-sided rectangle of that height as wide as holds the area,
	 * or, when the area is too small for that height (under `MIN_ASPECT` times its square), a square of that area,
	 * widened by `MIN_ASPECT`. Returns its width and height. Beads laid side by side at the pitch the slicer keeps
	 * (area / height) tile a solid layer with no gaps; a bead squashed into the layer is what the nozzle lays.
	 */
	public static function section(area:Float, height:Float):{width:Float, height:Float} {
		if (area >= height * height * MIN_ASPECT) return {width: area / height, height: height};
		var side = Math.sqrt(area);
		return {width: side * MIN_ASPECT, height: side};
	}

	function toolFor(width:Float, height:Float):Tool {
		// In whole quanta, a bead of at least two quanta high and two wider than high.
		var h = Math.max(2.0, Math.round(height / QUANTUM)), w = Math.max(h + 2.0, Math.round(width / QUANTUM));
		var key = w + ":" + h;
		var known = tools.get(key);
		if (known != null) return known;
		var tall = h * QUANTUM;
		var made = Tool.shaped(nextTool++, tall, CutterProfile.flat(w * QUANTUM, tall));
		tools.set(key, made);
		return made;
	}

	/**
	 * Lays the bead from where the tip was at the last call to `tip` (machine coordinates, metres, the tip's height
	 * above the build surface), given the filament the extruder has fed in all so far, in metres, and the layer's
	 * height. `opIndex` and `provenance` describe the program move under way, for colouring and picking.
	 *
	 * The extruder is a stepper: over one tick it feeds a few steps, and a bead sized from them alone would swell and
	 * thin by a quarter. The section is therefore the filament's volume per distance smoothed over `SMOOTHING` of
	 * travel, and what has been fed and not yet laid waits, so the volume laid is what was fed. Filament pushed while
	 * the tip stands still (a prime) piles up there as a blob when the tip moves off.
	 */
	public function follow(tip:Point3, fed:Float, height:Float, seconds:Float, opIndex:Int, provenance:Provenance):Void {
		if (Math.isNaN(lastFed)) lastFed = fed;
		var pushed = fed - lastFed;
		lastFed = fed;
		// The melt follows the filament like a spring: what flows out of the nozzle over this tick is a share of what
		// is stored behind it (the filament pushed and not yet out), which pressure advance in the plan cancels.
		var delta = pushed;
		if (meltLag > 0.0) {
			stored += pushed;
			delta = stored * (1.0 - Math.exp(-seconds / meltLag));
			stored -= delta;
		}
		// Pulling filament back takes what is waiting to be laid first, then what the nozzle is owed.
		var fedNow = 0.0;
		if (delta < 0.0) {
			var taken = Math.min(pending, -delta);
			pending -= taken;
			owed += -delta - taken;
		} else {
			var repaid = Math.min(owed, delta);
			owed -= repaid;
			fedNow = delta - repaid;
		}
		var start = last;
		last = tip;
		if (start == null) return;
		var dx = tip.x - start.x, dy = tip.y - start.y, dz = tip.z - start.z;
		var step = Math.sqrt(dx * dx + dy * dy + dz * dz);
		if (step < STILL) {
			standing += fedNow;
			return;
		}
		if (standing > 0.0 && height > 0.0) {
			blob(start, standing * filamentArea, height, opIndex, provenance);
			standing = 0.0;
		}
		pending += fedNow;
		density += (fedNow * filamentArea / step - density) * Math.min(1.0, step / SMOOTHING);
		var volume = Math.min(pending * filamentArea, density * step);
		// Nothing was fed over this stretch: it is a travel.
		if (volume < 1e-15 || !(height > 0.0)) return;
		pending -= volume / filamentArea;
		// Filament pushed into material already there has to go somewhere: it squeezes out beside the bead, which
		// widens until what was fed is laid. A bead that overlaps its neighbours, or whose rounded end lies in them,
		// is short of its volume, and the shortfall is made up by the beads that follow (one that spills out narrows
		// the next). Without this, a layer of beads laid at the pitch a slicer keeps would lose what they overlap.
		// The outer wall and the skirt face the air: their width is what was fed, and the dimensions of the print hold.
		var free = switch featureOfLine(provenance.line) {
			case ExternalPerimeter | Skirt: true;
			case _: false;
		};
		var wanted = free ? volume : volume + Math.max(-SQUEEZE * volume, Math.min(SQUEEZE * volume, shortfall));
		var shape = section(wanted / step, height);
		var added = lay(start, tip, shape.width, shape.height, opIndex, provenance);
		shortfall = Math.max(-LIMIT, Math.min(LIMIT, shortfall + volume - added));
	}

	/** Filament pushed while the tip stood: a disc of the layer's height that holds its volume, laid at `at`. */
	function blob(at:Point3, volume:Float, height:Float, opIndex:Int, provenance:Provenance):Void {
		if (volume < 1e-15) return;
		var radius = Math.sqrt(volume / (Math.PI * height));
		var added = lay(at, new Point3(at.x + STILL / 2.0, at.y, at.z), Math.max(2.0 * radius, height + 2.0 * QUANTUM), height, opIndex, provenance);
		shortfall = Math.max(-LIMIT, Math.min(LIMIT, shortfall + volume - added));
	}

	function lay(from:Point3, to:Point3, width:Float, height:Float, opIndex:Int, provenance:Provenance):Float {
		var tool = toolFor(width, height);
		// The bead lies below the tip: the tip is the top of the layer it is laying.
		var start = new Point3(from.x - center[0], from.y - center[1], from.z - height - center[2]);
		var end = new Point3(to.x - center[0], to.y - center[1], to.z - height - center[2]);
		var added = stock.add([new CutMove(tool, CutMotion.Path(PathGeometry.Line(start, end)), MoveKind.Cut, opIndex, provenance)]);
		deposited += added[0];
		beads++;
		meshes.markChanged();
		return added[0];
	}

	/** Filament fed and not yet laid or pulled back, in metres. */
	public function filamentPending():Float return pending;

	/** The tip jumps (a new pass starts somewhere else): the next bead starts afresh. */
	public function lift():Void {
		last = null;
		lastFed = Math.NaN;
		standing = 0.0;
		density = 0.0;
		shortfall = 0.0;
		stored = 0.0;
	}

	public function hasChanged():Bool return meshes.hasChanged();

	public function refreshPreview():Bool return meshes.refresh();

	public function takePreview():Null<Map<Int, GeometryData>> return meshes.take();

	public function previewChunks():Int return meshes.chunks();

	/** The whole print as it is now, as one mesh: waits for its contouring. */
	public function geometry():GeometryData return meshes.geometry();

	public function dispose():Void {
		meshes.dispose();
		stock.dispose();
	}
}
