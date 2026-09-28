package pickingstation;

/** Validated dimensions shared by the station layout and its example scenario. */
class PickingStationConfig {
	public static inline var FRAME_SIZE:Float = 20;
	public static inline var FOOT_HEIGHT:Float = 25;
	public static inline var BIN_GAP:Float = 20;
	public static inline var BIN_SIDE_CLEARANCE:Float = 20;
	public static inline var INDICATOR_DEPTH:Float = 24;

	public final binWidth:Float;
	public final binDepth:Float;
	public final binHeight:Float;
	public final binWallThickness:Float;
	public final binsPerShelf:Int;
	public final shelfCount:Int;
	public final shelfSpacing:Float;
	public final rackDepth:Float;
	public final benchWidth:Float;
	public final benchHeight:Float;
	public final benchDepth:Float;
	public final shelfInclinationDegrees:Float;

	public function new(binWidth:Float = 220, binDepth:Float = 300, binHeight:Float = 180,
		binWallThickness:Float = 3, binsPerShelf:Int = 3, shelfCount:Int = 2,
		shelfSpacing:Float = 300, rackDepth:Float = 380, benchWidth:Float = 900,
		benchHeight:Float = 850, benchDepth:Float = 600, shelfInclinationDegrees:Float = 0) {
		for (value in [binWidth, binDepth, binHeight, binWallThickness, shelfSpacing, rackDepth,
			benchWidth, benchHeight, benchDepth])
			if (!finitePositive(value)) throw "Picking-station dimensions must be finite and positive";
		if (binsPerShelf < 1 || shelfCount < 1) throw "Picking station needs bins and shelves";
		if (binWallThickness * 2 >= Math.min(binWidth, Math.min(binDepth, binHeight)))
			throw "Bin wall thickness leaves no usable bin envelope";
		if (!Math.isFinite(shelfInclinationDegrees) || Math.abs(shelfInclinationDegrees) > 30)
			throw "Shelf inclination must be between -30 and 30 degrees";
		this.binWidth = binWidth;
		this.binDepth = binDepth;
		this.binHeight = binHeight;
		this.binWallThickness = binWallThickness;
		this.binsPerShelf = binsPerShelf;
		this.shelfCount = shelfCount;
		this.shelfSpacing = shelfSpacing;
		this.rackDepth = rackDepth;
		this.benchWidth = benchWidth;
		this.benchHeight = benchHeight;
		this.benchDepth = benchDepth;
		this.shelfInclinationDegrees = shelfInclinationDegrees;
		if (rackDepth < binDepth + 70)
			throw "Rack depth does not leave clearance around the bins";
		var inclination = Math.abs(shelfInclinationDegrees) * Math.PI / 180;
		if (shelfSpacing <= binHeight * Math.cos(inclination) + binDepth * Math.sin(inclination) + 50)
			throw "Shelf spacing does not leave bin clearance";
		if (benchHeight <= 100 || benchWidth <= 100 || benchDepth <= 100)
			throw "Bench dimensions are too small";
		}

	public static function defaults():PickingStationConfig return new PickingStationConfig();

	public var rackWidth(get, never):Float;
	function get_rackWidth():Float
		return binsPerShelf * binWidth + (binsPerShelf - 1) * BIN_GAP + 2 * BIN_SIDE_CLEARANCE + 2 * FRAME_SIZE;

	public var usableShelfWidth(get, never):Float;
	function get_usableShelfWidth():Float return rackWidth - 2 * FRAME_SIZE;

	public var shelfWidth(get, never):Float;
	function get_shelfWidth():Float return rackWidth - 2 * FRAME_SIZE;

	public var shelfDepth(get, never):Float;
	function get_shelfDepth():Float return rackDepth - 2 * FRAME_SIZE;

	public var rackHeight(get, never):Float;
	function get_rackHeight():Float return FOOT_HEIGHT + shelfSpacing * (shelfCount + 1);

	public function shelfZ(shelfIndex:Int):Float {
		if (shelfIndex < 1 || shelfIndex > shelfCount) throw 'Shelf index $shelfIndex is outside 1..$shelfCount';
		return FOOT_HEIGHT + shelfSpacing * shelfIndex;
	}

	public function binX(binIndex:Int):Float {
		if (binIndex < 1 || binIndex > binsPerShelf) throw 'Bin index $binIndex is outside 1..$binsPerShelf';
		var first = -usableShelfWidth / 2 + BIN_SIDE_CLEARANCE + binWidth / 2;
		return first + (binIndex - 1) * (binWidth + BIN_GAP);
	}

	public function binY():Float return -rackDepth / 2 + FRAME_SIZE +
		ShelfAssembly.RETAINING_LIP_THICKNESS + 2 + binDepth / 2;
	public function indicatorY():Float return -rackDepth / 2 - INDICATOR_DEPTH / 2 - 4;

	public function positionId(shelfIndex:Int, binIndex:Int):String
		return 'rack-01/shelf-${twoDigits(shelfIndex)}/bin-${twoDigits(binIndex)}';

	public function indicatorId(shelfIndex:Int, binIndex:Int):String
		return positionId(shelfIndex, binIndex) + "/indicator";

	static function twoDigits(value:Int):String return value < 10 ? "0" + value : Std.string(value);
	static function finitePositive(value:Float):Bool return Math.isFinite(value) && value > 0;
}
